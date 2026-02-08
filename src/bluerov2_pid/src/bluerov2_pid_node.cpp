#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>

#include "bluerov2_pid/bluerov2_pid.hpp"
#include "bluerov2_pid/bluerov2_pid_node.hpp"

namespace bluerov2_pid {

PIDControllerNode::PIDControllerNode(
    const rclcpp::NodeOptions &options)
    : PIDControllerNode("PID Controller", options) {}

PIDControllerNode::PIDControllerNode(
    const std::string &node_name, const rclcpp::NodeOptions &options)
    : LifecycleNode(node_name, options),
      state_topic_name_{declare_parameter<std::string>("state_topic_name", "bluerov/odom")},
      ref_topic_name_{declare_parameter<std::string>("ref_topic_name", "bluerov2/references")},
      input_topic_name_{declare_parameter<std::string>("input_topic_name",
                                                     "/bluerov2/thrust")},
      publish_period_(std::chrono::milliseconds{
          declare_parameter<std::uint16_t>("publish_period_ms", 10)}),
      controller_(PIDController(
          declare_parameter<std::vector<double>>("controller.configuration_gains", std::vector<double>{10., 10., 10.}),
          declare_parameter<std::vector<double>>("controller.derivative_gains", std::vector<double>{15., 15., 15., 5., 5., 5.}),
          declare_parameter<std::vector<double>>("controller.integral_gains", std::vector<double>{.2, .2, .2, .15, .15, .15}),
          std::chrono::milliseconds{publish_period_},
          declare_parameter<std::vector<double>>("controller.int_state_lower_bounds", std::vector<double>{-15., -15., -30., -10., -10., -10.0}),
          declare_parameter<std::vector<double>>("controller.int_state_upper_bounds", std::vector<double>{15., 15., 120., 10., 10., 10.0}),
          declare_parameter<double>("controller.delta", 0.5236),
          declare_parameter<double>("controller.attitude_gain",10.)))
      {

        unsigned int thruster_angle_cfg = static_cast<unsigned int>(declare_parameter<int>("thruster_config_angle", 45));

        RCLCPP_INFO_STREAM(get_logger(), "Setting thruster angle to: " << thruster_angle_cfg);

        controller_.set_horizontal_thruster_angle_deg(thruster_angle_cfg);

        RCLCPP_INFO_STREAM(get_logger(), "Create subscription to state topic: " << ref_topic_name_);
        create_state_subscription();
        RCLCPP_INFO_STREAM(get_logger(), "Create subscription to ref topic: " << state_topic_name_);
        create_ref_subscription();
        RCLCPP_INFO_STREAM(get_logger(), "Create publisher to input topic: " << input_topic_name_);
        create_controller_publisher();
        RCLCPP_INFO_STREAM(get_logger(), "Create state timer callback with period (ms): " << publish_period_.count());
        create_state_timer_callback();
        RCLCPP_INFO_STREAM(get_logger(), "PID Node initialized");
  // controller_ptr_.set(std::make_shared<HystereticController>(controller_));
}

void PIDControllerNode::initialize_reference_from_state() {
  if (received_state_msg_ptr_ == nullptr) {
    RCLCPP_WARN_ONCE(
        get_logger(),
        "Cannot initialize reference from state: no state message received.");
    return;
  }
 
  auto ref_msg = std::make_shared<bluerov_interfaces::msg::Reference>();
  ref_msg->header.stamp = received_state_msg_ptr_->header.stamp;
  ref_msg->pos.x = received_state_msg_ptr_->pose.pose.position.x;
  ref_msg->pos.y = received_state_msg_ptr_->pose.pose.position.y;
  ref_msg->pos.z = received_state_msg_ptr_->pose.pose.position.z + 0.2;
  ref_msg->quat = received_state_msg_ptr_->pose.pose.orientation;
  ref_msg->velocity = geometry_msgs::msg::Twist();  // zero velocity
  ref_msg->acceleration = geometry_msgs::msg::Twist();  // zero accel
 
  received_ref_msg_ptr_ = ref_msg;
}

void PIDControllerNode::create_state_subscription() {
  auto on_state_message = [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
    // received_state_msg_ptr_.set(std::move(msg));
    received_state_msg_ptr_ = msg;
    // RCLCPP_INFO(get_logger(), "RECEIVED STATE MSG");
    // update state with observer measurements
    // Eigen::Quaterniond q(
    //     msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // Eigen::VectorXd euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    // Eigen::Vector3d eta, nu;
    // eta << msg->pose.pose.position.x, msg->pose.pose.position.y,
    //     euler(2);
    // nu << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    //     msg->twist.twist.angular.z;

    // controller_.set_measurements(eta, nu);
  };

  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_name_, rclcpp::QoS(1), on_state_message);
}

void PIDControllerNode::create_ref_subscription()
{
  auto on_ref_message =
      [this](const bluerov_interfaces::msg::Reference::SharedPtr msg) {
        // received_ref_msg_ptr_.set(std::move(msg));
            // std::shared_ptr<bluerov_interfaces::msg::Reference> last_ref_msg;
    // received_ref_msg_ptr_.get(last_ref_msg);
    received_ref_msg_ptr_ = msg;

    get_params();

    // if (last_ref_msg == nullptr) {
    //   RCLCPP_WARN(get_logger(), "Received reference message was a nullptr.");
    //   return;
    // }

    // Vector3d eta_ref, eta_dot_ref, eta_ddot_ref;
    // eta_ref << msg->configuration.x, msg->configuration.y, msg->configuration.z;
    // eta_dot_ref << msg->velocity.x, msg->velocity.y, msg->velocity.z;
    // eta_ddot_ref << msg->acceleration.x, msg->acceleration.y,
    //     msg->acceleration.z;

    // controller_.set_references(eta_ref, eta_dot_ref, eta_ddot_ref);
      };

  ref_sub_ = this->create_subscription<bluerov_interfaces::msg::Reference>(
      ref_topic_name_, rclcpp::QoS(1), on_ref_message);
}

void PIDControllerNode::create_controller_publisher() {
  pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>(
      input_topic_name_, rclcpp::QoS(1));

  // realtime_pub_ = std::make_shared<
  //     realtime_tools::RealtimePublisher<bluerov_interfaces::msg::ActuatorInput>>(
  //     pub_);

  controller_state_pub_ = this->create_publisher<bluerov_interfaces::msg::ControllerState>(
    "/bluerov/controller/state", rclcpp::QoS(1));
    // controller_state_pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>(
    // state_topic_name_,
    // rclcpp::QoS(10).deadline(deadline_duration_),
    // sensor_publisher_options);
}

void PIDControllerNode::create_state_timer_callback() {
  auto timer_callback = [this]() {
    const auto current_time = this->get_clock()->now();

    std::shared_ptr<nav_msgs::msg::Odometry> last_msg = received_state_msg_ptr_;

    if (last_msg == nullptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Received observer message was a nullptr.");
      return;
    }

    // if (current_time - last_msg->header.stamp > timeout_) {
    //   return;
    // }

    // update state with observer measurements
    Eigen::Quaterniond quat(
        last_msg->pose.pose.orientation.w, last_msg->pose.pose.orientation.x,
        last_msg->pose.pose.orientation.y, last_msg->pose.pose.orientation.z);


    Eigen::Vector3d pos;
    Vector6d nu;
    pos << last_msg->pose.pose.position.x, last_msg->pose.pose.position.y,
        last_msg->pose.pose.position.z;
    nu << last_msg->twist.twist.linear.x, last_msg->twist.twist.linear.y,
        last_msg->twist.twist.linear.z, last_msg->twist.twist.angular.x,
        last_msg->twist.twist.angular.y, last_msg->twist.twist.angular.z;

    controller_.set_measurements(pos, quat, nu);

    // update references
    std::shared_ptr<bluerov_interfaces::msg::Reference> last_ref_msg = received_ref_msg_ptr_;

    if (last_ref_msg == nullptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Received reference message was a nullptr.");
      return;
    }

    Eigen::Vector3d pos_ref;
    Eigen::Quaterniond quat_ref(last_ref_msg->quat.w, last_ref_msg->quat.x,
                         last_ref_msg->quat.y, last_ref_msg->quat.z);

    Vector6d nu_ref,
    nu_dot_ref;

    pos_ref << last_ref_msg->pos.x,
        last_ref_msg->pos.y, last_ref_msg->pos.z;
    nu_ref << last_ref_msg->velocity.linear.x, last_ref_msg->velocity.linear.y,
        last_ref_msg->velocity.linear.z, last_ref_msg->velocity.angular.x, 
        last_ref_msg->velocity.angular.y, last_ref_msg->velocity.angular.z;
    nu_dot_ref << last_ref_msg->acceleration.linear.x,
        last_ref_msg->acceleration.linear.y,
        last_ref_msg->acceleration.linear.z,
        last_ref_msg->acceleration.angular.x,
        last_ref_msg->acceleration.angular.y,
        last_ref_msg->acceleration.angular.z;

    controller_.set_references(pos_ref, quat_ref, nu_ref, nu_dot_ref);

    // const auto current_time = this->get_clock()->now();
    // if (realtime_pub_->trylock()) {
      controller_.update();
      Vector8d control_inputs = controller_.get_control_input();
      
      // auto &message = pub_->msg_;
      bluerov_interfaces::msg::ActuatorInput message;
      message.header.stamp = current_time;
      message.thrust1 = control_inputs(0);
      message.thrust2 = control_inputs(1);
      message.thrust3 = control_inputs(2);
      message.thrust4 = control_inputs(3);
      message.thrust5 = control_inputs(4);
      message.thrust6 = control_inputs(5);
      message.thrust7 = control_inputs(6);
      message.thrust8 = control_inputs(7);
      pub_->publish(message);

      // pub_->unlockAndPublish();
    // }

    Eigen::Vector<double, 6> xi_hat = controller_.get_integral_state();
    controller_state_message_.header.stamp = current_time;
    controller_state_message_.q = controller_.get_switching_state();

    controller_state_message_.bias_x = xi_hat(0);
    controller_state_message_.bias_y = xi_hat(1);
    controller_state_message_.bias_z = xi_hat(2);
    controller_state_message_.bias_ang1 = xi_hat(3);
    controller_state_message_.bias_ang2 = xi_hat(4);
    controller_state_message_.bias_ang3 = xi_hat(5);

    controller_state_pub_->publish(controller_state_message_);
  };
  timer_ = this->create_wall_timer(publish_period_, timer_callback);
  timer_->cancel();
}

void PIDControllerNode::get_params() {
  std::vector<double> Kp;
  std::vector<double> Kd;
  std::vector<double> Ki;
  double attitude_gain;
  std::vector<double> lower_bounds;
  std::vector<double> upper_bounds;

  get_parameter("controller.configuration_gains", Kp );
  get_parameter("controller.derivative_gains", Kd);
  get_parameter("controller.integral_gains", Ki);
  get_parameter("controller.attitude_gain", attitude_gain);
  get_parameter("controller.int_state_lower_bounds", lower_bounds);
  get_parameter("controller.int_state_upper_bounds", upper_bounds);

  controller_.set_gains(Kp, Kd, Ki, attitude_gain);
  controller_.set_integral_limits(lower_bounds, upper_bounds);
}


CallbackReturn
PIDControllerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PIDControllerNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_->on_activate();
  controller_state_pub_->on_activate();
  initialize_reference_from_state();
  timer_->reset();
  controller_.clear_integral_state();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PIDControllerNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  pub_->on_deactivate();
  controller_state_pub_->on_deactivate();
  timer_->cancel();
  controller_.clear_integral_state();
  received_ref_msg_ptr_ = nullptr;
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn
PIDControllerNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
PIDControllerNode::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

} // namespace bluerov2_pid

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bluerov2_pid::PIDControllerNode)
// RCLCPP_COMPONENTS_REGISTER_NODE(ShipObserverNode)
