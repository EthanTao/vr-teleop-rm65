#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <std_srvs/srv/trigger.hpp>
namespace std_msgs { namespace msg { struct String { std::string data; }; }}
namespace sensor_msgs { namespace msg { struct JointState {
using SharedPtr=std::shared_ptr<JointState>;
struct { struct { int sec=0; unsigned int nanosec=0; } stamp; std::string frame_id; } header;
std::vector<std::string> name; std::vector<double> position;
}; }}
namespace rclcpp {
inline bool ok() { return true; }
struct QoS { explicit QoS(int) {} };
struct TimerBase { using SharedPtr=std::shared_ptr<TimerBase>; };
template<class T> struct Publisher { using SharedPtr=std::shared_ptr<Publisher<T>>; bool fail=false; std::vector<T> rows; void publish(const T& row){ if(fail) throw std::runtime_error("mock publish failure"); rows.push_back(row); } };
template<class T> struct Service { using SharedPtr=std::shared_ptr<Service<T>>; };
template<class T> struct Subscription { using SharedPtr=std::shared_ptr<Subscription<T>>; };
struct Node {
bool allow_servo_motion=false;
template<class T> T declare_parameter(const std::string&, const T&) { return static_cast<T>(allow_servo_motion); }
using Request=std_srvs::srv::Trigger::Request; using Response=std_srvs::srv::Trigger::Response;
using Callback=std::function<void(std::shared_ptr<Request>,std::shared_ptr<Response>)>;
std::unordered_map<std::string,Callback> services;
std::function<void(sensor_msgs::msg::JointState::SharedPtr)> command;
std::function<void()> timer;
template<class T> typename Publisher<T>::SharedPtr create_publisher(const std::string&,QoS) { return std::make_shared<Publisher<T>>(); }
template<class T,class F> typename Service<T>::SharedPtr create_service(const std::string& name,F cb) { services[name]=cb; return std::make_shared<Service<T>>(); }
template<class T,class F> typename Subscription<T>::SharedPtr create_subscription(const std::string&,QoS,F cb) { command=cb; return std::make_shared<Subscription<T>>(); }
template<class D,class F> TimerBase::SharedPtr create_wall_timer(D,F cb) { timer=cb; return std::make_shared<TimerBase>(); }
}; }
struct rm_realtime_arm_joint_state_t {
int errCode=0;
struct { double joint_position[7]{}; double joint_speed[7]{}; int joint_err_code[7]{}; bool joint_en_flag[7]{}; } joint_status;
struct { unsigned int err_len=0; int err[24]{}; } err;
};
int arm_dof_g=6; bool udp_joint_speed_state_g=true; void* robot_handle=nullptr;
struct Api {
int sends=0, soft=0, hard=0, hard_result=0;
std::atomic<bool> block{false}, entered{false};
int rm_set_arm_slow_stop(void*) { ++soft; return 0; }
int rm_set_arm_stop(void*) { ++hard; return hard_result; }
int rm_movej_canfd(void*,float*,bool,float,int,int) { ++sends; entered=true; while(block.load()) std::this_thread::yield(); return 0; }
} Rm_Api;
#include "native_contract_under_test.h"
using namespace rm65_servo_contract;
int main() {
  rclcpp::Node readonly_node; setup(&readonly_node);
  assert(!motion_allowed && readonly_node.services.empty() && !readonly_node.command && !readonly_node.timer);
  shutdown(); assert(Rm_Api.hard==0 && Rm_Api.soft==0 && Rm_Api.sends==0);
  rclcpp::Node node; node.allow_servo_motion=true; setup(&node);
  auto call=[&](const std::string& name) {
    auto response=std::make_shared<rclcpp::Node::Response>();
    node.services.at("rm_driver/"+name)(std::make_shared<rclcpp::Node::Request>(),response);
    return response;
  };
  auto command=[&](unsigned long long gen) {
    auto message=std::make_shared<sensor_msgs::msg::JointState>();
    message->header.frame_id=std::to_string(gen);
    auto now=clock_ns(CLOCK_REALTIME);
    message->header.stamp.sec=now/1000000000LL; message->header.stamp.nanosec=now%1000000000LL;
    for(int i=0;i<6;++i){message->name.push_back("joint"+std::to_string(i+1));message->position.push_back(0.0);}
    return message;
  };
  node.command(command(0)); assert(Rm_Api.sends==0);
  assert(!call("servo_enable_stream")->success);
  rm_realtime_arm_joint_state_t packet; for(int i=0;i<6;++i)packet.joint_status.joint_en_flag[i]=true;
  on_packet(packet);
  assert(call("servo_slow_stop")->success); assert(Rm_Api.soft==1);
  auto enabled=call("servo_enable_stream"); assert(enabled->success);
  auto old_generation=stream_generation.load();
  node.command(command(old_generation)); assert(Rm_Api.sends==1);
  assert(call("servo_hard_stop")->success); assert(!stream_enabled.load());
  on_packet(packet); assert(call("servo_enable_stream")->success);
  node.command(command(old_generation)); assert(Rm_Api.sends==1);
  node.command(command(stream_generation.load())); assert(Rm_Api.sends==2);
  // A stop closes/increments the generation while a command is still inside
  // the serialized SDK call; it cannot report confirmation until that call exits.
  Rm_Api.block=true; Rm_Api.entered=false;
  auto pending=command(stream_generation.load());
  std::thread sender([&](){node.command(pending);});
  while(!Rm_Api.entered.load()) std::this_thread::yield();
  std::thread stopper([&](){assert(call("servo_hard_stop")->success);});
  while(stream_enabled.load()) std::this_thread::yield();
  Rm_Api.block=false; sender.join(); stopper.join();
  assert(!stream_enabled.load());
  on_packet(packet); assert(call("servo_enable_stream")->success);
  int count=Rm_Api.sends; node.command(pending); assert(Rm_Api.sends==count);
  feedback_publisher->fail=true; on_packet(packet);
  assert(feedback_publish_fault.load() && stream_enabled.load());
  int failed_publish_hard=Rm_Api.hard; node.timer();
  assert(!stream_enabled.load() && Rm_Api.hard==failed_publish_hard+1);
  assert(!call("servo_enable_stream")->success);
  feedback_publisher->fail=false; on_packet(packet);
  assert(call("servo_enable_stream")->success);
  // A dead bridge cannot keep native streaming armed indefinitely.
  last_command_ns.store(clock_ns(CLOCK_MONOTONIC)-300000000LL);
  int hard_before=Rm_Api.hard; node.timer();
  assert(!stream_enabled.load() && Rm_Api.hard==hard_before+1);
  Rm_Api.hard_result=-1; assert(!call("servo_hard_stop")->success);
  on_packet(packet); assert(!call("servo_enable_stream")->success);
  Rm_Api.hard_result=0;
  assert(call("servo_hard_stop")->success); on_packet(packet); assert(call("servo_enable_stream")->success);
  int connection_hard_before=Rm_Api.hard;
  on_connection_fault(); node.timer();
  assert(!stream_enabled.load() && connection_fault.load() && Rm_Api.hard==connection_hard_before+1);
  on_packet(packet); assert(!call("servo_enable_stream")->success);
  int shutdown_hard_before=Rm_Api.hard; shutdown();
  assert(!stream_enabled.load() && Rm_Api.hard==shutdown_hard_before+1);
  std::cout << "native mock: startup lock, confirmed soft/hard stop, generation replay, serialized stop, watchdog, failed-stop lock passed\n";
  std::cout << feedback_publisher->rows.back().data << "\n" << enabled->message << "\n";
}
