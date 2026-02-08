
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "bluerov2_teleop/bluerov2_teleop_node.hpp"
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>
namespace bluerov2_teleop {

BlueROVTeleOpNode::BlueROVTeleOpNode(const rclcpp::NodeOptions &options)
    : Node("bluerov2_teleop_node", options),
      odom_topic_name_(declare_parameter<std::string>(
          "odometry_topic_name", "/bluerov/observer/odom")),
      publish_period_{std::chrono::milliseconds{
          declare_parameter<std::uint16_t>("publish_period_ms", 10)}},
      horiz_gain_(declare_parameter<double>("horizontal_vel_scaling", 1.0)),
      yaw_gain_(declare_parameter<double>("yaw_rate_scaling", 1.0)),
      orientation_gain_(declare_parameter<double>("orientation_gain", 1.0)),
      depth_gain_(declare_parameter<double>("depth_scaling", 1.0)),
      depth_prop_gain_(declare_parameter<double>("depth_prop_gain", 0.0)),
      depth_joy_gain_(declare_parameter<double>("depth_joy_gain", 8.4)),
      horiz_joy_gain_(declare_parameter<double>("horiz_joy_gain", 5.0)),
      yaw_joy_gain_(declare_parameter<double>("yaw_joy_gain", 1.2)),
      integrator_gain_(declare_parameter<double>("integral_depth_gain", 0.0)),
      velocity_gains_(declare_parameter<std::vector<double>>(
          "velocity_gains", std::vector<double>{10, 10, 0, 0, 0, 10})),
      velocity_control_(declare_parameter<bool>("velocity_control", false)),
      depth_setpoint_{0.0}, velocity_refs_{Eigen::Vector4d::Zero()},
      control_inputs_{Vector8d::Zero()}, nu_{Vector6d::Zero()},
      nu_d_{Vector6d::Zero()}, dV_{Vector6d::Zero()}, tau_{Vector6d::Zero()},
      horizontal_inputs_{Eigen::Vector2d::Zero()},
      rotation_inputs_{Eigen::Vector2d::Zero()}, z_int_{0.0} {
  unsigned int thruster_angle_cfg = static_cast<unsigned int>(
      declare_parameter<int>("thruster_config_angle", 45));

  RCLCPP_INFO_STREAM(get_logger(),
                     "Setting thruster angle to: " << thruster_angle_cfg);

  set_horizontal_thruster_angle_deg(thruster_angle_cfg);

  Eigen::Matrix<double, 3, 8> LENGTHS_THRUSTERS;
  LENGTHS_THRUSTERS << 0.156, 0.156, -0.156, -0.156, 0.12, 0.12, -0.12, -0.12,
      0.111, -0.111, 0.111, -0.111, 0.218, -0.218, 0.218, -0.218, 0.085, 0.085,
      0.085, 0.085, 0, 0, 0, 0;

  Eigen::Vector3d e_1 = {1, 0, 0};
  Eigen::Vector3d e_3 = {0, 0, 1};

  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d thrust_dir =
        MIXER[i] * Sophus::SO3d::rotZ(kPi / 180 * ALPHA[i]).matrix() * e_1;
    B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i));
  }
  for (int i = 4; i < 8; i++) {
    Eigen::Vector3d thrust_dir = -MIXER[i] * e_3;
    B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Thrust configuration matrix: \n" << B_);

  B_pinv_ = B_.completeOrthogonalDecomposition().pseudoInverse();

  actuator_pub_ =
      this->create_publisher<bluerov_interfaces::msg::ActuatorInput>(
          "/bluerov/u", rclcpp::QoS(1));

  Kd_ = Eigen::Map<Vector6d>(velocity_gains_.data(), 6).asDiagonal();
  k_int_ << 0, 0, integrator_gain_, 0, 0, 0;

  dt_ = publish_period_.count() / (1000.0);

  input_msg_.thrust1 = 0;
  input_msg_.thrust2 = 0;
  input_msg_.thrust3 = 0;
  input_msg_.thrust4 = 0;
  input_msg_.thrust5 = 0;
  input_msg_.thrust6 = 0;
  input_msg_.thrust7 = 0;
  input_msg_.thrust8 = 0;

  create_joy_subscription();
  create_odom_subscription();
  create_state_timer_callback();
}

void BlueROVTeleOpNode::create_joy_subscription() {
  auto on_joy_message = [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
    received_joy_msg_ptr_.set(std::move(msg));
    std::shared_ptr<sensor_msgs::msg::Joy> last_joy_msg;
    received_joy_msg_ptr_.get(last_joy_msg);

    if (last_joy_msg->buttons[0] == 1 && !enable_depth_controller_) {
      RCLCPP_INFO(get_logger(), "Enabling depth controller..");

      enable_depth_controller_ = true;
      std::shared_ptr<nav_msgs::msg::Odometry> last_odom_msg;
      received_odom_msg_ptr_.get(last_odom_msg);
      if (last_odom_msg == nullptr) {
        RCLCPP_WARN(
            get_logger(),
            "Last odom msg was a nullptr! Setting desired depth to 0.5");
        depth_setpoint_ = 0.5;
      } else {
        depth_setpoint_ = last_odom_msg->pose.pose.position.z;
      }
    }
    if (last_joy_msg->buttons[1] && enable_depth_controller_) {
      RCLCPP_INFO(get_logger(), "Disabling depth controller..");
      enable_depth_controller_ = false;
    }

    // if (last_joy_msg->axes[7] == 1 && !last_msg_depth_inc_) {
    //   depth_setpoint_ += 0.1;
    //   RCLCPP_INFO_STREAM(this->get_logger(),
    //                      "Depth setpoint: " << depth_setpoint_);
    // }
    // if (last_joy_msg->axes[7] == -1 && !) {
    //   depth_setpoint_ -= 0.1;
    //   RCLCPP_INFO_STREAM(this->get_logger(),
    //                      "Depth setpoint: " << depth_setpoint_);
    // }
  };

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(1), on_joy_message);
}

void BlueROVTeleOpNode::create_odom_subscription() {
  auto on_odom_msg = [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
    received_odom_msg_ptr_.set(std::move(msg));
  };

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_name_, rclcpp::QoS(1), on_odom_msg);
}

void BlueROVTeleOpNode::create_state_timer_callback() {
  int counter = 0;
  auto timer_callback = [this, &counter]() {
    if (counter % 100 == 0) {
      get_ros_params();
      // RCLCPP_INFO(get_logger(), "Updating ros params");
    }
    counter++;

    const auto current_time = this->get_clock()->now();

    std::shared_ptr<sensor_msgs::msg::Joy> last_msg;
    received_joy_msg_ptr_.get(last_msg);

    if (last_msg == nullptr) {
      // RCLCPP_WARN(get_logger(), "Received joy message was a nullptr!");
      rclcpp::Clock &clock = *this->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), clock, 5000,
                                  "Waiting to recieve joy message");
      return;
    }

    // Eigen::Vector2d horizontal_inputs;
    horizontal_inputs_ << last_msg->axes[4],
        -last_msg->axes[3]; // left joy x-axis and right joy y-axis
    // Eigen::Vector2d rotation_inputs;
    rotation_inputs_ << last_msg->axes[2], last_msg->axes[5]; // L2 and R2

    velocity_refs_.head(2) = horiz_gain_ * horizontal_inputs_;
    velocity_refs_(2) = -last_msg->axes[1] * depth_gain_;
    velocity_refs_(3) =
        yaw_gain_ * (rotation_inputs_(0) - rotation_inputs_(1)) / 2.0;

    if (velocity_control_) {
      std::shared_ptr<nav_msgs::msg::Odometry> last_odom_msg;
      received_odom_msg_ptr_.get(last_odom_msg);

      if (last_odom_msg == nullptr) {
        z_int_ = 0;
        return;
      }

       Eigen::Quaterniond quat(
        last_odom_msg->pose.pose.orientation.w, last_odom_msg->pose.pose.orientation.x,
        last_odom_msg->pose.pose.orientation.y, last_odom_msg->pose.pose.orientation.z);


      double yaw = atan2((2 * (quat.x() * quat.y() + quat.z() * quat.w())), (1 - 2 * (pow(quat.y(), 2) + pow(quat.z(), 2))));
      Eigen::Quaterniond quat_d = Eigen::Quaterniond(cos(yaw/2.0), 0.0, 0.0, sin(yaw/2.0)) ;
      // quat = quat_yaw;

      Eigen::Quaterniond quat_e = quat_d.inverse() * quat;

      Eigen::Vector3d dV_ang;
      dV_ang << orientation_gain_ * quat_e.vec();


      nu_ << last_odom_msg->twist.twist.linear.x,
          last_odom_msg->twist.twist.linear.y,
          last_odom_msg->twist.twist.linear.z,
          last_odom_msg->twist.twist.angular.x,
          last_odom_msg->twist.twist.angular.y,
          last_odom_msg->twist.twist.angular.z;

      nu_d_ << velocity_refs_(0), velocity_refs_(1), 0, 0, 0, velocity_refs_(3);

      if (enable_depth_controller_) {
        dV_ << 0, 0,
            depth_prop_gain_ *
                (last_odom_msg->pose.pose.position.z - depth_setpoint_),
            dV_ang;

        // control_inputs_ = -B_.completeOrthogonalDecomposition().solve(
        //     dV_ + k_int_ * z_int_ + Kd_ * (nu_ - nu_d_));
        // Vector6d temp = dV_ + k_int_ * z_int_ + Kd_ * (nu_ - nu_d_);
        control_inputs_ =
            -B_pinv_ * (dV_ + k_int_ * z_int_ + Kd_ * (nu_ - nu_d_));

        z_int_ += dt_ * (last_odom_msg->pose.pose.position.z - depth_setpoint_);
      } else {
        nu_d_ << velocity_refs_(0), velocity_refs_(1), velocity_refs_(2), 0, 0,
            velocity_refs_(3);
        control_inputs_ = -B_pinv_ * (Kd_ * (nu_ - nu_d_));
      }

    } else {
      if (enable_depth_controller_) {
        std::shared_ptr<nav_msgs::msg::Odometry> last_odom_msg;
        received_odom_msg_ptr_.get(last_odom_msg);
        
        // Bugfix: Where you get segfault if no odometry but try to enter this mode by pressing A button
        // Now instead of segfault just exit before segfault and continue driving
        if (!last_odom_msg) {
          z_int_ = 0.0;
          return;
        }

        // vertical_input_ = -last_msg->axes[1] - integrator_gain_ * z_int_;
        vertical_input_ =
            -depth_prop_gain_ *
                (last_odom_msg->pose.pose.position.z - depth_setpoint_) -
            integrator_gain_ * z_int_;
        z_int_ += dt_ * (last_odom_msg->pose.pose.position.z - depth_setpoint_);
      } else {
        vertical_input_ = -last_msg->axes[1];
      }
      tau_ << horiz_joy_gain_ * horizontal_inputs_,
          depth_joy_gain_ * vertical_input_, 0, 0,
          yaw_joy_gain_ * (rotation_inputs_(0) - rotation_inputs_(1)) / 2.0;

      control_inputs_ = B_pinv_ * tau_;
    }
    input_msg_.header.stamp = current_time;
    input_msg_.thrust1 = control_inputs_(0);
    input_msg_.thrust2 = control_inputs_(1);
    input_msg_.thrust3 = control_inputs_(2);
    input_msg_.thrust4 = control_inputs_(3);
    input_msg_.thrust5 = control_inputs_(4);
    input_msg_.thrust6 = control_inputs_(5);
    input_msg_.thrust7 = control_inputs_(6);
    input_msg_.thrust8 = control_inputs_(7);

    actuator_pub_->publish(input_msg_);
  };
  timer_ = this->create_wall_timer(publish_period_, timer_callback);
  // timer_->cancel();
}

void BlueROVTeleOpNode::set_horizontal_thruster_angle_deg(
    unsigned int angle_deg) {
  ALPHA[0] = -angle_deg;
  ALPHA[1] = angle_deg;
  ALPHA[2] = -180.0 + angle_deg;
  ALPHA[3] = 180.0 - angle_deg;
}

void BlueROVTeleOpNode::get_ros_params() {
  bool velocity_control;
  std::vector<double> velocity_gains;

  get_parameter("velocity_control", velocity_control);
  if (velocity_control != velocity_control_) {
    velocity_control_ = velocity_control;
    if (velocity_control_) {
      RCLCPP_INFO(get_logger(), "Velocity control enabled");
    } else {
      RCLCPP_INFO(get_logger(), "Velocity control disabled");
    }
  }

  get_parameter("velocity_gains", velocity_gains);
  if (velocity_gains.size() != 6) {
    RCLCPP_WARN(get_logger(), "Invalid velocity gains");
  } else {
    velocity_gains_ = velocity_gains;
  }

  get_parameter("horiz_gain", horiz_gain_);
  get_parameter("yaw_gain", yaw_gain_);
  get_parameter("orientation_gain", orientation_gain_);
  get_parameter("depth_gain", depth_gain_);
  get_parameter("depth_prop_gain", depth_prop_gain_);
  get_parameter("depth_joy_gain", depth_joy_gain_);
  get_parameter("horiz_joy_gain", horiz_joy_gain_);
  get_parameter("integrator_gain", integrator_gain_);
}

} // namespace bluerov2_teleop
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(bluerov2_teleop::BlueROVTeleOpNode)
