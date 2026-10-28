// Copyright 2026 RM65 teleoperation contributors
// SPDX-License-Identifier: Apache-2.0
// Lifecycle/validity adapter for MoveIt Servo's existing Humble PoseTracking.
// This file implements no IK, PID, or joint trajectory interpolation.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
using namespace std::chrono_literals;

template <typename T>
T parameter(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& fallback)
{
  if (!node->has_parameter(name))
    return node->declare_parameter<T>(name, fallback);
  return node->get_parameter(name).get_value<T>();
}

double secondsSince(TimePoint then, TimePoint now)
{
  return std::chrono::duration<double>(now - then).count();
}

std::string escaped(const std::string& value)
{
  std::ostringstream out;
  for (unsigned char character : value)
  {
    switch (character)
    {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (character < 0x20)
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(character) << std::dec;
        else
          out << character;
    }
  }
  return out.str();
}

class PoseTrackingSession : public std::enable_shared_from_this<PoseTrackingSession>
{
public:
  PoseTrackingSession(const rclcpp::Node::SharedPtr& node,
                      const moveit_servo::ServoParameters::SharedConstPtr& servo_parameters,
                      const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
    : node_(node), servo_parameters_(servo_parameters), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    if (std::string(node_->get_namespace()) != "/")
      throw std::runtime_error("rm65_pose_tracking must run in the global namespace for target isolation");
    // Humble's moveToPose() uses ROS time during its initial bounded wait. A
    // paused simulation clock can make that wait unbounded, so the mock-hardware
    // simulation deliberately uses the same advancing wall/ROS clock as hardware.
    if (parameter<bool>(node_, "use_sim_time", false))
      throw std::runtime_error("PoseTracking adapter requires use_sim_time=false");
    clock_guard_ = node_->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto& value : parameters)
      {
        if (value.get_name() == "use_sim_time" &&
            (value.get_type() != rclcpp::ParameterType::PARAMETER_BOOL || value.as_bool()))
        {
          result.successful = false;
          result.reason = "PoseTracking requires an advancing clock; use_sim_time must remain false";
          break;
        }
      }
      return result;
    });

    timeout_s_ = positiveParameter("input_timeout_s", 0.20);
    feedback_timeout_s_ = positiveParameter("feedback_timeout_s", 0.20);
    future_tolerance_s_ = positiveParameter("future_stamp_tolerance_s", 0.02);
    max_linear_velocity_ = positiveParameter("max_linear_velocity_m_s", 0.05);
    max_angular_velocity_ = positiveParameter("max_angular_velocity_rad_s", 0.25);
    positional_tolerance_ = positiveParameter("position_tolerance_m", 0.0005);
    angular_tolerance_ = positiveParameter("orientation_tolerance_rad", 0.005);
    planning_frame_ = servo_parameters_->planning_frame;
    tcp_frame_ = servo_parameters_->ee_frame_name;
    joint_names_ = parameter<std::vector<std::string>>(
        node_, "joint_names", { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" });
    if (joint_names_.size() != 6 || std::set<std::string>(joint_names_.begin(), joint_names_.end()).size() != 6)
      throw std::runtime_error("joint_names must contain six unique RM65 arm joints");
    if (servo_parameters_->robot_link_command_frame != tcp_frame_)
      throw std::runtime_error("PoseTracking requires robot_link_command_frame == ee_frame_name (tcp)");
    if (servo_parameters_->command_in_type != "speed_units")
      throw std::runtime_error("PoseTracking adapter requires command_in_type=speed_units");

    // The P-only error bound below is a command constraint before the upstream
    // component. It is not a guarantee about delayed physical feedback/actuation.
    const std::vector<std::string> axes{ "x", "y", "z" };
    max_position_gain_ = 0.0;
    for (const auto& axis : axes)
    {
      const auto prefix = "moveit_servo." + axis;
      max_position_gain_ = std::max(max_position_gain_, positiveParameter(prefix + "_proportional_gain", 1.5));
      requireZeroGain(prefix + "_integral_gain");
      requireZeroGain(prefix + "_derivative_gain");
    }
    angular_gain_ = positiveParameter("moveit_servo.angular_proportional_gain", 0.5);
    requireZeroGain("moveit_servo.angular_integral_gain");
    requireZeroGain("moveit_servo.angular_derivative_gain");
    positiveParameter("moveit_servo.windup_limit", 0.05);

    status_publisher_ = node_->create_publisher<std_msgs::msg::String>("/rm65/pose_tracking/status", 32);
    validated_target_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/rm65/internal/target_pose", rclcpp::QoS(1));

    tracker_ = std::make_unique<moveit_servo::PoseTracking>(node_, servo_parameters_, planning_scene_monitor);
    tracker_->servo_->setPaused(true);
    tracker_->stopMotion();
    tracker_->resetTargetPose();
    resume_not_before_ = SteadyClock::now() + quietInterval();
  }

  void start()
  {
    const std::weak_ptr<PoseTrackingSession> weak = shared_from_this();
    // Keep this executable in the global namespace, remapping only /target_pose
    // to /rm65/internal/target_pose. The external absolute topic stays distinct.
    const auto target_topic = parameter<std::string>(node_, "target_input_topic", "/rm65/target_pose");
    const auto feedback_topic = parameter<std::string>(node_, "feedback_topic", "/joint_states");
    target_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic, rclcpp::QoS(1),
        [weak](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          if (const auto self = weak.lock())
            self->targetCallback(*message);
        });
    enable_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/rm65/enable", rclcpp::QoS(1),
        [weak](std_msgs::msg::Bool::ConstSharedPtr message) {
          if (const auto self = weak.lock())
            self->enableCallback(message->data);
        });
    feedback_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        feedback_topic, rclcpp::SensorDataQoS().keep_last(1),
        [weak](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          if (const auto self = weak.lock())
            self->feedbackCallback(*message);
        });
    watchdog_ = node_->create_wall_timer(5ms, [weak] {
      if (const auto self = weak.lock())
        self->watchdog();
    });
    status_timer_ = node_->create_wall_timer(200ms, [weak] {
      if (const auto self = weak.lock())
      {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->publishStatusLocked();
      }
    });
    worker_ = std::thread([this] { run(); });
  }

  ~PoseTrackingSession() { shutdown(); }

  void failClosed(const char* reason) noexcept
  {
    try
    {
      std::lock_guard<std::mutex> lock(mutex_);
      failClosedLocked(reason);
    }
    catch (...)
    {
      // Even an exceptional lock failure must not prevent the independent
      // upstream pause/stop operations from being attempted by a thread exit.
      pauseLocked();
    }
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closing_)
        return;
      closing_ = true;
      requested_ = false;
      ++epoch_;
      reason_ = "shutdown";
      pauseLocked();
    }
    condition_.notify_all();
    if (worker_.joinable())
      worker_.join();
    for (const auto& timer : { watchdog_, status_timer_ })
    {
      if (timer)
      {
        try
        {
          timer->cancel();
        }
        catch (const rclcpp::exceptions::RCLError&)
        {
          // The ROS context may already be invalid; motion is paused and the
          // worker has been joined before timer cancellation is attempted.
        }
      }
    }
  }

private:
  double positiveParameter(const std::string& name, double fallback)
  {
    const double value = parameter<double>(node_, name, fallback);
    if (!std::isfinite(value) || value <= 0)
      throw std::runtime_error(name + " must be finite and positive");
    return value;
  }

  void requireZeroGain(const std::string& name)
  {
    const double value = parameter<double>(node_, name, 0.0);
    if (!std::isfinite(value) || value != 0.0)
      throw std::runtime_error(name + " must be zero for the declared velocity error bound");
  }

  SteadyClock::duration quietInterval() const noexcept
  {
    // Old upstream Twist messages must age out before a *new Grip epoch* may
    // unpause. A zero Twist alone is not an acknowledgement of queue drainage.
    return std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(servo_parameters_->incoming_command_timeout +
                                      2.0 * servo_parameters_->publish_period));
  }

  bool stampFresh(const builtin_interfaces::msg::Time& stamp, double limit) const noexcept
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u)
      return false;
    try
    {
      const rclcpp::Time time(stamp, node_->get_clock()->get_clock_type());
      if (time.nanoseconds() <= 0)
        return false;
      const double age = (node_->now() - time).seconds();
      return std::isfinite(age) && age >= -future_tolerance_s_ && age <= limit;
    }
    catch (...)
    {
      // Invalid ROS timestamps and unavailable clocks invalidate the lease.
      return false;
    }
  }

  void pauseLocked() noexcept
  {
    // Only run() may unpause. Callback/watchdog calls cannot revive a worker.
    // Reassert stop while a worker is exiting: moveToPose() clears its own stop
    // flag at entry, but it never controls this independently held pause gate.
    try
    {
      tracker_->servo_->setPaused(true);
    }
    catch (...)
    {
      // Still attempt stopMotion if the pause operation itself fails.
    }
    try
    {
      tracker_->stopMotion();
    }
    catch (...)
    {
      // A SIGINT may already have invalidated the ROS context. Pause remains set.
    }
  }

  void failClosedLocked(const char* reason) noexcept
  {
    // Exception handlers never publish: a second publish/clock exception must
    // not escape the worker or executor thread, or recursively report a fault.
    pauseLocked();
    if (requested_ || active_)
      ++epoch_;
    requested_ = false;
    active_ = false;
    require_release_ = true;
    permanent_fault_ = true;
    boundary_sequence_ = target_sequence_;
    resume_not_before_ = SteadyClock::now() + quietInterval();
    try
    {
      reason_ = reason;
    }
    catch (...)
    {
      // Fault latches and motion inhibition do not depend on string allocation.
    }
    condition_.notify_all();
  }

  void inhibitLocked(const std::string& reason, bool require_release)
  {
    const bool changed = requested_ || active_ || reason_ != reason || (!require_release_ && require_release);
    if (requested_ || active_)
      ++epoch_;
    requested_ = false;
    require_release_ = require_release_ || require_release;
    boundary_sequence_ = target_sequence_;
    reason_ = reason;
    resume_not_before_ = SteadyClock::now() + quietInterval();
    pauseLocked();
    if (changed)
      publishStatusLocked();
    condition_.notify_all();
  }

  bool feedbackFreshLocked(TimePoint now) const
  {
    return have_feedback_ && secondsSince(feedback_received_, now) <= feedback_timeout_s_ &&
           stampFresh(last_feedback_stamp_, feedback_timeout_s_);
  }

  bool targetFreshLocked(TimePoint now) const
  {
    return have_target_ && target_sequence_ > boundary_sequence_ &&
           secondsSince(target_received_, now) <= timeout_s_ && stampFresh(target_.header.stamp, timeout_s_);
  }

  bool runnableLocked(TimePoint now) const
  {
    return requested_ && !require_release_ && !permanent_fault_ && !closing_ &&
           secondsSince(enable_received_, now) <= timeout_s_ && feedbackFreshLocked(now) && targetFreshLocked(now);
  }

  void enableCallback(bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_)
      return;
    enable_received_ = SteadyClock::now();
    if (!enabled)
    {
      require_release_ = false;
      inhibitLocked("grip_released", false);
      return;
    }
    if (require_release_ || permanent_fault_)
    {
      const std::string next_reason = permanent_fault_ ? "restart_required" : "release_required";
      if (next_reason != reason_)
      {
        reason_ = next_reason;
        publishStatusLocked();
      }
      return;
    }
    if (!requested_)
    {
      ++epoch_;
      boundary_sequence_ = target_sequence_;
      enable_started_ = enable_received_;
      reason_ = "waiting_for_new_target";
      requested_ = true;
      publishStatusLocked();
    }
    condition_.notify_all();
  }

  void feedbackCallback(const sensor_msgs::msg::JointState& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_)
      return;
    if (message.name.size() != 6 || message.position.size() != 6)
    {
      have_feedback_ = false;
      if (requested_ || active_)
        inhibitLocked("invalid_feedback_dimensions", true);
      return;
    }
    std::set<std::string> names(message.name.begin(), message.name.end());
    const bool valid = message.name.size() == message.position.size() && names.size() == message.name.size() &&
                       std::all_of(joint_names_.begin(), joint_names_.end(),
                                   [&names](const std::string& name) { return names.count(name) == 1; }) &&
                       std::all_of(message.position.begin(), message.position.end(),
                                   [](double value) { return std::isfinite(value); }) &&
                       stampFresh(message.header.stamp, feedback_timeout_s_);
    if (!valid)
    {
      have_feedback_ = false;
      if (requested_ || active_)
        inhibitLocked("invalid_feedback", true);
      return;
    }
    const rclcpp::Time stamp(message.header.stamp, node_->get_clock()->get_clock_type());
    const rclcpp::Time previous(last_feedback_stamp_, node_->get_clock()->get_clock_type());
    if (have_feedback_ && stamp <= previous)
    {
      if (stamp < previous && (requested_ || active_))
        inhibitLocked("feedback_time_regressed", true);
      return;  // Repeated/cached feedback never renews the lease.
    }
    last_feedback_stamp_ = message.header.stamp;
    feedback_received_ = SteadyClock::now();
    have_feedback_ = true;
    condition_.notify_all();
  }

  geometry_msgs::msg::PoseStamped constrainedTarget(const geometry_msgs::msg::PoseStamped& desired)
  {
    const auto transform = tf_buffer_.lookupTransform(planning_frame_, tcp_frame_, tf2::TimePointZero);
    if (!stampFresh(transform.header.stamp, feedback_timeout_s_))
      throw std::runtime_error("stale_tcp_transform");
    const Eigen::Vector3d current(transform.transform.translation.x, transform.transform.translation.y,
                                  transform.transform.translation.z);
    Eigen::Quaterniond current_q(transform.transform.rotation.w, transform.transform.rotation.x,
                                 transform.transform.rotation.y, transform.transform.rotation.z);
    if (!current.allFinite() || !current_q.coeffs().allFinite() || current_q.norm() < 1e-9)
      throw std::runtime_error("invalid_tcp_transform");
    current_q.normalize();
    const Eigen::Vector3d goal(desired.pose.position.x, desired.pose.position.y, desired.pose.position.z);
    Eigen::Vector3d error = goal - current;
    const double position_bound = max_linear_velocity_ / max_position_gain_;
    const double error_norm = error.stableNorm();
    if (!error.allFinite() || !std::isfinite(error_norm))
      throw std::runtime_error("invalid_target_error");
    const bool position_clamped = error_norm > position_bound;
    if (position_clamped)
      error *= position_bound / error_norm;
    Eigen::Quaterniond goal_q(desired.pose.orientation.w, desired.pose.orientation.x,
                              desired.pose.orientation.y, desired.pose.orientation.z);
    goal_q.normalize();
    Eigen::Quaterniond difference = goal_q * current_q.conjugate();
    if (difference.w() < 0)
      difference.coeffs() *= -1;
    const Eigen::AngleAxisd rotation(difference.normalized());
    const double rotation_bound = max_angular_velocity_ / angular_gain_;
    const bool orientation_clamped = rotation.angle() > rotation_bound;
    if (orientation_clamped)
      goal_q = Eigen::Quaterniond(Eigen::AngleAxisd(rotation_bound, rotation.axis())) * current_q;
    auto result = desired;  // Keep original ROS stamp; do not renew source freshness.
    result.pose.position.x = current.x() + error.x();
    result.pose.position.y = current.y() + error.y();
    result.pose.position.z = current.z() + error.z();
    result.pose.orientation.x = goal_q.x();
    result.pose.orientation.y = goal_q.y();
    result.pose.orientation.z = goal_q.z();
    result.pose.orientation.w = goal_q.w();
    position_clamped_ = position_clamped;
    orientation_clamped_ = orientation_clamped;
    if (position_clamped || orientation_clamped)
      ++clamped_targets_;
    return result;
  }

  void targetCallback(const geometry_msgs::msg::PoseStamped& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_)
      return;
    const auto& p = message.pose.position;
    const auto& q = message.pose.orientation;
    const double quaternion_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (message.header.frame_id != planning_frame_ || !stampFresh(message.header.stamp, timeout_s_) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 0.001)
    {
      ++rejected_targets_;
      if (requested_ || active_)
        inhibitLocked("invalid_target", true);
      return;
    }
    const rclcpp::Time stamp(message.header.stamp, node_->get_clock()->get_clock_type());
    const rclcpp::Time previous(target_.header.stamp, node_->get_clock()->get_clock_type());
    if (have_target_ && stamp <= previous)
    {
      ++rejected_targets_;
      if (stamp < previous && (requested_ || active_))
        inhibitLocked("target_time_regressed", true);
      return;
    }
    target_ = message;
    have_target_ = true;
    target_received_ = SteadyClock::now();
    ++target_sequence_;
    if (active_ && worker_epoch_ == epoch_ && runnableLocked(target_received_))
    {
      try
      {
        validated_target_publisher_->publish(constrainedTarget(target_));
      }
      catch (const std::exception& exception)
      {
        inhibitLocked(std::string("target_transform_failure: ") + exception.what(), true);
      }
    }
    condition_.notify_all();
  }

  void watchdog()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_)
    {
      pauseLocked();
      return;
    }
    const auto now = SteadyClock::now();
    if (requested_)
    {
      if (secondsSince(enable_received_, now) > timeout_s_)
        inhibitLocked("enable_timeout", true);
      else if (!feedbackFreshLocked(now))
        inhibitLocked("feedback_timeout", true);
      else if (!targetFreshLocked(now) && secondsSince(enable_started_, now) > timeout_s_)
        inhibitLocked("target_timeout", true);
    }
    if (!requested_ || require_release_ || (active_ && worker_epoch_ != epoch_))
      pauseLocked();
    condition_.notify_all();
  }

  void run() noexcept
  {
    try
    {
      runLoop();
    }
    catch (...)
    {
      // This also covers an exception from wait/predicate/lock operations
      // outside moveToPose's narrower try block.
      failClosed("worker_exception");
    }
  }

  void runLoop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!closing_)
    {
      condition_.wait_for(lock, 5ms, [this] {
        return closing_ || (runnableLocked(SteadyClock::now()) && SteadyClock::now() >= resume_not_before_ &&
                            target_sequence_ > completed_sequence_);
      });
      if (closing_)
        break;
      if (!runnableLocked(SteadyClock::now()) || SteadyClock::now() < resume_not_before_ ||
          target_sequence_ <= completed_sequence_)
        continue;
      worker_epoch_ = epoch_;
      const auto started_epoch = epoch_;
      const auto started_sequence = target_sequence_;
      try
      {
        geometry_msgs::msg::PoseStamped validated_target;
        try
        {
          validated_target = constrainedTarget(target_);
        }
        catch (const std::exception& exception)
        {
          inhibitLocked(std::string("target_transform_failure: ") + exception.what(), true);
          continue;
        }
        tracker_->resetTargetPose();
        validated_target_publisher_->publish(validated_target);
        active_ = true;
        reason_ = "tracking";
        if (!publishStatusLocked())
          continue;  // Publication failure already latched fault and paused.
        if (!rclcpp::ok() || !runnableLocked(SteadyClock::now()) || started_epoch != epoch_)
        {
          active_ = false;
          pauseLocked();
          continue;
        }
        // The only unpause site is protected by the same lock as false/timeout.
        tracker_->servo_->setPaused(false);
        lock.unlock();
        const auto result = tracker_->moveToPose(
            Eigen::Vector3d::Constant(positional_tolerance_), angular_tolerance_, timeout_s_);
        lock.lock();
        pauseLocked();
        active_ = false;
        tracker_->resetTargetPose();
        completed_sequence_ = started_sequence;
        last_tracking_result_ = static_cast<int>(result);
        if (started_epoch != epoch_ || !requested_ || require_release_)
          resume_not_before_ = SteadyClock::now() + quietInterval();
        else if (result == moveit_servo::PoseTrackingStatusCode::SUCCESS)
          reason_ = "at_target";
        else
          inhibitLocked("pose_tracking_aborted", true);
        publishStatusLocked();
      }
      catch (...)
      {
        if (!lock.owns_lock())
          lock.lock();
        failClosedLocked("tracker_exception");
      }
    }
    pauseLocked();
  }

  bool publishStatusLocked() noexcept
  {
    if (!rclcpp::ok())
    {
      failClosedLocked("ros_context_shutdown");
      return false;
    }
    try
    {
      std::ostringstream out;
      out << "{\"status_sequence\":" << ++status_sequence_ << ",\"state\":\"" << escaped(reason_)
          << "\",\"requested\":" << (requested_ ? "true" : "false")
          << ",\"active\":" << (active_ ? "true" : "false")
          << ",\"release_required\":" << (require_release_ ? "true" : "false")
          << ",\"restart_required\":" << (permanent_fault_ ? "true" : "false")
          << ",\"epoch\":" << epoch_ << ",\"target_sequence\":" << target_sequence_
          << ",\"tracking_result\":" << last_tracking_result_
          << ",\"position_clamped\":" << (position_clamped_ ? "true" : "false")
          << ",\"orientation_clamped\":" << (orientation_clamped_ ? "true" : "false")
          << ",\"clamped_targets\":" << clamped_targets_ << ",\"rejected_targets\":" << rejected_targets_ << "}";
      std_msgs::msg::String message;
      message.data = out.str();
      status_publisher_->publish(message);
      return true;
    }
    catch (const rclcpp::exceptions::RCLError&)
    {
      // rclcpp::ok() and publish() race with shutdown. Fail closed without
      // attempting another diagnostic publication through the invalid context.
      failClosedLocked("status_publish_failure");
    }
    catch (...)
    {
      failClosedLocked("status_serialization_failure");
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr clock_guard_;
  std::unique_ptr<moveit_servo::PoseTracking> tracker_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr validated_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_, status_timer_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  double timeout_s_{ 0.2 }, feedback_timeout_s_{ 0.2 }, future_tolerance_s_{ 0.02 };
  double max_linear_velocity_{ 0.05 }, max_angular_velocity_{ 0.25 };
  double max_position_gain_{ 1.5 }, angular_gain_{ 0.5 };
  double positional_tolerance_{ 0.0005 }, angular_tolerance_{ 0.005 };
  std::string planning_frame_, tcp_frame_;
  std::vector<std::string> joint_names_;
  bool closing_{ false }, requested_{ false }, active_{ false }, require_release_{ true }, permanent_fault_{ false };
  bool have_target_{ false }, have_feedback_{ false }, position_clamped_{ false }, orientation_clamped_{ false };
  uint64_t epoch_{ 0 }, worker_epoch_{ 0 }, target_sequence_{ 0 }, boundary_sequence_{ 0 }, completed_sequence_{ 0 };
  uint64_t clamped_targets_{ 0 }, rejected_targets_{ 0 }, status_sequence_{ 0 };
  int last_tracking_result_{ -1 };
  std::string reason_{ "awaiting_release" };
  geometry_msgs::msg::PoseStamped target_;
  builtin_interfaces::msg::Time last_feedback_stamp_;
  TimePoint target_received_{}, feedback_received_{}, enable_received_{}, enable_started_{}, resume_not_before_{};
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // makeServoParameters() declares its parameters itself on Humble. Automatically
  // declaring overrides would cause ParameterAlreadyDeclaredException.
  const auto node = std::make_shared<rclcpp::Node>("rm65_pose_tracking");
  // Humble's MultiThreadedExecutor starts additional unguarded std::threads;
  // exceptions there would bypass our outer spin catch. PoseTracking/Servo's
  // motion work already runs independently from this callback executor.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::shared_ptr<PoseTrackingSession> session;
  std::shared_ptr<PoseTrackingSession> executor_session;
  std::atomic<bool> executor_failed{ false };
  std::thread executor_thread([&executor, &executor_session, &executor_failed]() noexcept {
    try
    {
      executor.spin();
    }
    catch (...)
    {
      // Callbacks may throw while reading clocks or publishing. No exception
      // may escape a std::thread entry and bypass the normal shutdown/join path.
      executor_failed.store(true);
      if (const auto current = std::atomic_load(&executor_session))
        current->failClosed("executor_exception");
      try
      {
        rclcpp::shutdown();
      }
      catch (...)
      {
        // The failure flag also wakes main if ROS shutdown itself fails.
      }
    }
  });
  int result = 0;
  try
  {
    if (parameter<bool>(node, "use_sim_time", false))
      throw std::runtime_error("PoseTracking adapter requires an advancing clock: use_sim_time=false");
    const auto parameters = moveit_servo::ServoParameters::makeServoParameters(node, "moveit_servo", false);
    if (!parameters)
      throw std::runtime_error("invalid moveit_servo parameters");
    auto monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    if (!monitor->getPlanningScene() || !monitor->getRobotModel()->hasJointModelGroup(parameters->move_group_name))
      throw std::runtime_error("robot description/SRDF has no configured RM65 planning group");
    monitor->startSceneMonitor();
    monitor->startStateMonitor(parameters->joint_topic);
    monitor->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                      false);
    monitor->providePlanningSceneService();
    if (!monitor->waitForCurrentRobotState(node->now(), 10.0))
      throw std::runtime_error("timed out waiting for current joint state");
    session = std::make_shared<PoseTrackingSession>(node, parameters, monitor);
    std::atomic_store(&executor_session, session);
    session->start();
    RCLCPP_INFO(node->get_logger(), "MoveIt PoseTracking ready; Grip release and new fresh target are required");
    while (rclcpp::ok() && !executor_failed.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    session->shutdown();
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(node->get_logger(), "PoseTracking adapter stopped: %s", exception.what());
    result = 1;
    if (session)
      session->shutdown();
  }
  try
  {
    executor.cancel();
  }
  catch (...)
  {
    // cancel() also triggers a ROS guard condition and can race shutdown.
    // The session has already stopped; invalidate the context to end spin.
    result = 1;
    try
    {
      rclcpp::shutdown();
    }
    catch (...)
    {
    }
  }
  if (executor_thread.joinable())
    executor_thread.join();
  if (executor_failed.load())
    result = 1;
  std::atomic_store(&executor_session, std::shared_ptr<PoseTrackingSession>{});
  session.reset();  // Destroy subscriptions only after all executor callbacks finish.
  if (rclcpp::ok())
    rclcpp::shutdown();
  return result;
}
