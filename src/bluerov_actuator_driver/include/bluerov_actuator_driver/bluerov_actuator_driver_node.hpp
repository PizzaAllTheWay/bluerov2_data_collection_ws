#ifndef BLUEROV_ACTUATOR_DRIVER__BLUEROV_ACTUATOR_DRIVER_NODE_HPP_
#define BLUEROV_ACTUATOR_DRIVER__BLUEROV_ACTUATOR_DRIVER_NODE_HPP_

#include <chrono>
#include <climits>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/battery_state.hpp"

#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "bluerov_interfaces/msg/camera_tilt.hpp"
#include "bluerov_interfaces/msg/light_brightness.hpp"
#include "bluerov_interfaces/msg/thruster_pwm.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "sensor_msgs/msg/joy.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"

#include "bluerov_actuator_driver/bluerov_actuator_driver.hpp"
#include "bluerov_actuator_driver/visibility_control.hpp"

using namespace std::chrono_literals;

namespace bluerov_actuator_driver {
// using CallbackReturn =
// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status =
        future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class BlueROVActuatorDriverNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  BLUEROV_ACTUATOR_DRIVER_PUBLIC
  explicit BlueROVActuatorDriverNode(const rclcpp::NodeOptions& options);

  BLUEROV_ACTUATOR_DRIVER_PUBLIC
  explicit BlueROVActuatorDriverNode(
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void create_input_subscription();
  void create_joy_subscription();
  void create_cam_tilt_subscription();
  void create_light_brightness_subscription();
  void create_timer_callback();

  bool change_pid_state(std::uint8_t transition,
                        std::chrono::seconds time_out = 3s);

  // void log_state();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state activating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state deactivating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state cleaningup
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state shutting down
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state) override;

  const std::string input_topic_name_;
  const std::string joy_topic_name_;
  const std::string cam_tilt_topic_name_;
  const std::string brightness_topic_name_;
  const std::string node_change_state_topic = "/bluerov2_pid/change_state";

  bool enable_topic_stats_;
  const std::string topic_stats_topic_name_;
  std::chrono::milliseconds topic_stats_publish_period_;
  std::chrono::milliseconds deadline_duration_;
  std::chrono::milliseconds update_period_;

  BlueROVActuatorDriver bluerov_actuator_driver_;

  std::shared_ptr<rclcpp::Subscription<bluerov_interfaces::msg::ActuatorInput>>
      input_sub_;
  realtime_tools::RealtimeBox<
      std::shared_ptr<bluerov_interfaces::msg::ActuatorInput>>
      received_inputs_msg_ptr_{nullptr};

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
  realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::Joy>>
      received_joy_msg_ptr_{nullptr};

  std::shared_ptr<rclcpp::Subscription<bluerov_interfaces::msg::CameraTilt>>
      cam_tilt_sub_;

  std::shared_ptr<
      rclcpp::Subscription<bluerov_interfaces::msg::LightBrightness>>
      brightness_sub_;

  std::shared_ptr<
      rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>>
      battery_voltage_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      bluerov_interfaces::msg::ThrusterPWM>>
      pwm_pub_;

  std::vector<double> control_inputs_;

  rclcpp::TimerBase::SharedPtr timer_;

  uint32_t num_missed_deadlines_sub_;
  bool last_aux_inc_{false};

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
      client_change_pid_state_;
};
}  // namespace bluerov_actuator_driver

#endif  // BlueROV_ACTUATOR_DRIVER__BlueROV_ACTUATOR_DRIVER_NODE_HPP_
