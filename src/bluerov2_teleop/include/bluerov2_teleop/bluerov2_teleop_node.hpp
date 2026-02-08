#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joy.hpp"
#include "sophus/geometry.hpp"

namespace bluerov2_teleop {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;

const double kPi = Sophus::Constants<double>::pi();

class BlueROVTeleOpNode : public rclcpp::Node {
public:
  explicit BlueROVTeleOpNode(const rclcpp::NodeOptions &options);

  // virtual ~BlueROVTeleOpNode();

  std::vector<int> MIXER = {1, 1, -1, -1, 1, -1, -1, 1};
  std::vector<int> ALPHA = {-45, 45, -135, 135};

private:
  void create_state_timer_callback();
  void create_joy_subscription();
  void create_odom_subscription();
  void set_horizontal_thruster_angle_deg(unsigned int angle_deg);
  void get_ros_params();

  const std::string input_topic_name_{"/bluerov/u"};
  const std::string odom_topic_name_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds publish_period_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
  realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::Joy>>
      received_joy_msg_ptr_{nullptr};
  std::shared_ptr<rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput>>
      actuator_pub_;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;
  realtime_tools::RealtimeBox<std::shared_ptr<nav_msgs::msg::Odometry>>
      received_odom_msg_ptr_{nullptr};

  Eigen::MatrixXd B_{6, 8};
  // Eigen::Matrix<double, 8, 6> B_pinv_;
  Eigen::MatrixXd B_pinv_;

  Eigen::DiagonalMatrix<double, 6> Kd_;
  Vector6d k_int_;

  double vertical_input_{0};
  double horiz_gain_;
  double yaw_gain_;
  double orientation_gain_;
  double depth_gain_;
  double depth_prop_gain_;
  double depth_joy_gain_;
  double horiz_joy_gain_;
  double yaw_joy_gain_;
  double integrator_gain_;
  std::vector<double> velocity_gains_;
  bool enable_depth_controller_{false};
  bool velocity_control_{false};

  double depth_setpoint_;
  Eigen::Vector4d velocity_refs_;
  Vector8d control_inputs_;
  Vector6d nu_;
  Vector6d nu_d_;
  Vector6d dV_;
  Vector6d tau_;
  Eigen::Vector2d horizontal_inputs_;
  Eigen::Vector2d rotation_inputs_;
  double z_int_; // integrator state in z
  double dt_;

  bluerov_interfaces::msg::ActuatorInput input_msg_;
};

} // namespace bluerov2_teleop