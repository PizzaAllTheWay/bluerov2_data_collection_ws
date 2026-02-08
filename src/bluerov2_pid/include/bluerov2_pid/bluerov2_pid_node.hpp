#ifndef PID_CONTROLLER__PID_CONTROLLER_NODE_HPP_
#define PID_CONTROLLER__PID_CONTROLLER_NODE_HPP_


#include <string>
#include <climits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rcutils/logging_macros.h"

#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "bluerov_interfaces/msg/controller_state.hpp"
#include "bluerov_interfaces/msg/reference.hpp"

// #include "realtime_tools/realtime_box.h"
// #include "realtime_tools/realtime_publisher.h"

#include "bluerov2_pid/bluerov2_pid.hpp"

namespace bluerov2_pid
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn;

class PIDControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PIDControllerNode(const rclcpp::NodeOptions & options);

  explicit PIDControllerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // void init_state_message();

  void create_state_subscription();

  void create_ref_subscription();

  void create_controller_publisher();

  void create_state_timer_callback();

  void get_params();

  void initialize_reference_from_state(); 
  
  // void log_controller_state();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state activating
  /// \param[in] lifecycle node state
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state deactivating
  /// \param[in] lifecycle node state
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state cleaningup
  /// \param[in] lifecycle node state
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state shutting down
  /// \param[in] lifecycle node state
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  const std::string state_topic_name_;
  const std::string ref_topic_name_;
  const std::string input_topic_name_{"/bluerov2/u"};
      // const std::string controller_topic_name_;
      // bool enable_topic_stats_;
      // const std::string topic_stats_topic_name_;
      // std::chrono::milliseconds topic_stats_publish_period_;
      // std::chrono::milliseconds deadline_duration_;
      std::chrono::milliseconds publish_period_;

  PIDController controller_;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> state_sub_;
  // realtime_tools::RealtimeBox<std::shared_ptr<nav_msgs::msg::Odometry>>
      // received_state_msg_ptr_{nullptr};
  std::shared_ptr<nav_msgs::msg::Odometry> received_state_msg_ptr_{nullptr};

  std::shared_ptr<rclcpp::Subscription<bluerov_interfaces::msg::Reference>> ref_sub_;
  // realtime_tools::RealtimeBox<std::shared_ptr<bluerov_interfaces::msg::Reference>>
      // received_ref_msg_ptr_{nullptr};
  std::shared_ptr<bluerov_interfaces::msg::Reference> received_ref_msg_ptr_{nullptr};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      bluerov_interfaces::msg::ActuatorInput>> pub_;

  // std::shared_ptr<realtime_tools::RealtimePublisher<bluerov_interfaces::msg::ActuatorInput>>
  // realtime_pub_{nullptr};


  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        bluerov_interfaces::msg::ControllerState>> controller_state_pub_;
        
  bluerov_interfaces::msg::ControllerState controller_state_message_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::milliseconds timeout_{500};

  // uint32_t num_missed_deadlines_pub_;
  // uint32_t num_missed_deadlines_sub_;
};
}
#endif //  PID_CONTROLLER__PID_CONTROLLER_NODE_HPP_
