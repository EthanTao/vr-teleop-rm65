// Native XR SDK input port. Motion generation belongs exclusively to MoveIt Servo.
// SDK callback API reference: XR-Robotics/XRoboToolkit-Teleop-ROS (Apache-2.0).
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "PXREARobotSDK.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using Publisher = rclcpp::Publisher<std_msgs::msg::String>;
std::mutex callback_mutex;
std::weak_ptr<Publisher> raw_publisher;
bool accepting_callbacks = false;
std::size_t maximum_json_bytes = 131072;

std::optional<std::string> bounded_json(const char* value, std::size_t capacity)
{
  if (value == nullptr)
    return std::nullopt;
  // The native header defines stateJson as char[16352]. Never search beyond
  // that actual field if a malformed callback omits its NUL terminator.
  const auto limit = std::min(capacity, maximum_json_bytes + 1);
  const auto end = std::find(value, value + limit, '\0');
  const auto length = static_cast<std::size_t>(end - value);
  if (end == value + limit || length == 0 || length > maximum_json_bytes)
    return std::nullopt;
  return std::string(value, length);
}

void publish_event(const Publisher::SharedPtr& publisher, const char* event)
{
  std_msgs::msg::String message;
  message.data = std::string("{\"sdk_event\":\"") + event + "\"}";
  publisher->publish(message);
}

void sdk_callback(void* context, PXREAClientCallbackType type, int status, void* user_data)
{
  (void)context;
  (void)status;  // SDK callback status is not a documented controller-tracking bitmask.
  std::lock_guard<std::mutex> lock(callback_mutex);
  if (!accepting_callbacks)
    return;
  const auto publisher = raw_publisher.lock();
  if (!publisher || !rclcpp::ok())
    return;
  try
  {
    switch (type)
    {
      case PXREADeviceStateJson:
      {
        if (user_data == nullptr)
        {
          publish_event(publisher, "invalid_payload");
          break;
        }
        const auto& state = *static_cast<PXREADevStateJson*>(user_data);
        const auto json = bounded_json(state.stateJson, sizeof(state.stateJson));
        if (!json)
        {
          publish_event(publisher, "invalid_payload");
          break;
        }
        std_msgs::msg::String message;
        message.data = *json;  // Preserve the device clock and all quality fields.
        publisher->publish(message);
        break;
      }
      case PXREAServerDisconnect:
        publish_event(publisher, "server_disconnected");
        break;
      case PXREADeviceMissing:
        publish_event(publisher, "device_missing");
        break;
      default:
        break;
    }
  }
  catch (...)
  {
    // Never allow a C++ exception to cross a native SDK callback boundary.
    // The source stops advancing and the independent input watchdog disarms.
  }
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("xr_sdk_source");
  const auto topic = node->declare_parameter<std::string>("raw_topic", "/rm65/xr/raw");
  const auto publisher = node->create_publisher<std_msgs::msg::String>(
      topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  {
    std::lock_guard<std::mutex> lock(callback_mutex);
    raw_publisher = publisher;
    accepting_callbacks = true;
  }
  bool initialized = false;
  int result = 0;
  try
  {
    if (PXREAInit(nullptr, sdk_callback, PXREAFullMask) != 0)
      throw std::runtime_error("PXREAInit failed");
    initialized = true;
    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "XR SDK input stopped: %s", error.what());
    result = 2;
  }
  {
    std::lock_guard<std::mutex> lock(callback_mutex);
    accepting_callbacks = false;
    raw_publisher.reset();
  }
  // Release callback state before deinit; native deinit may wait for worker callbacks.
  if (initialized && PXREADeinit() != 0)
  {
    RCLCPP_ERROR(node->get_logger(), "PXREADeinit reported a failure");
    result = 2;
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return result;
}
