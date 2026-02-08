// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <bluerov_interfaces/msg/detail/camera_tilt__struct.hpp>
#include <chrono>
#include <cstdint>
#include <rclcpp/parameter_value.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <string>
#include <vector>

#include "bluerov_actuator_driver/bluerov_actuator_driver_node.hpp"

namespace bluerov_actuator_driver {

BlueROVActuatorDriverNode::BlueROVActuatorDriverNode(
    const rclcpp::NodeOptions& options)
    : BlueROVActuatorDriverNode("bluerov_actuator_driver", options) {}

BlueROVActuatorDriverNode::BlueROVActuatorDriverNode(
    const std::string& node_name, const rclcpp::NodeOptions& options)
    : LifecycleNode(node_name, options),
      input_topic_name_(
          declare_parameter<std::string>("input_topic_name", "/bluerov/u")),
      joy_topic_name_(declare_parameter<std::string>("joy_topic_name", "/joy")),
      cam_tilt_topic_name_(declare_parameter<std::string>(
          "cam_tilt_topic_name", "/bluerov/camera/tilt")),
      brightness_topic_name_(declare_parameter<std::string>(
          "brightness_topic_name", "/bluerov/light/brightness")),
      enable_topic_stats_(
          declare_parameter<bool>("enable_topic_stats", "False")),
      topic_stats_topic_name_(declare_parameter<std::string>(
          "topic_stats_topic_name", "actuator_driver_stats")),
      deadline_duration_(std::chrono::milliseconds{
          declare_parameter<int>("deadline_duration_ms", 0)}),
      update_period_(std::chrono::milliseconds{
          declare_parameter<int>("update_period_ms", 1000)}),
      bluerov_actuator_driver_{},
      control_inputs_{0, 0, 0, 0, 0, 0, 0, 0},
      num_missed_deadlines_sub_{0U} {
  client_change_pid_state_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>(
          node_change_state_topic);

  battery_voltage_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/bluerov/battery", rclcpp::QoS(1));
    
  pwm_pub_ = this->create_publisher<bluerov_interfaces::msg::ThrusterPWM>(
      "/bluerov/thruster_pwm", rclcpp::QoS(1));

  create_input_subscription();
  create_joy_subscription();
  create_cam_tilt_subscription();
  create_light_brightness_subscription();
  create_timer_callback();
}

void BlueROVActuatorDriverNode::create_input_subscription() {
  rclcpp::SubscriptionOptions input_subscription_options;
  input_subscription_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo&) -> void {
    num_missed_deadlines_sub_++;
  };
  if (enable_topic_stats_) {
    input_subscription_options.topic_stats_options.state =
        rclcpp::TopicStatisticsState::Enable;
    input_subscription_options.topic_stats_options.publish_topic =
        topic_stats_topic_name_;
    input_subscription_options.topic_stats_options.publish_period =
        topic_stats_publish_period_;
  }
  auto on_input_message =
      [this](const bluerov_interfaces::msg::ActuatorInput::SharedPtr msg) {
        control_inputs_[0] = msg->thrust1;
        control_inputs_[1] = msg->thrust2;
        control_inputs_[2] = msg->thrust3;
        control_inputs_[3] = msg->thrust4;
        control_inputs_[4] = msg->thrust5;
        control_inputs_[5] = msg->thrust6;
        control_inputs_[6] = msg->thrust7;
        control_inputs_[7] = msg->thrust8;

        bluerov_actuator_driver_.process_control_inputs(control_inputs_);

        std::vector<uint32_t> thruster_pwm_values =
            bluerov_actuator_driver_.get_thruster_pwm();
        bluerov_interfaces::msg::ThrusterPWM pwm_msg;
        // Set message header
        pwm_msg.header.stamp = this->now();
        pwm_msg.header.frame_id = "bluerov_base_link";

        // Set PWM values
        pwm_msg.pwm1 = thruster_pwm_values[0];
        pwm_msg.pwm2 = thruster_pwm_values[1];
        pwm_msg.pwm3 = thruster_pwm_values[2];
        pwm_msg.pwm4 = thruster_pwm_values[3];
        pwm_msg.pwm5 = thruster_pwm_values[4];
        pwm_msg.pwm6 = thruster_pwm_values[5];
        pwm_msg.pwm7 = thruster_pwm_values[6];
        pwm_msg.pwm8 = thruster_pwm_values[7];

        pwm_pub_->publish(pwm_msg);

        bluerov_actuator_driver_.send_pwm();
      };
  input_sub_ =
      this->create_subscription<bluerov_interfaces::msg::ActuatorInput>(
          input_topic_name_, rclcpp::QoS(1), on_input_message);
  // input_sub_ =
  // this->create_subscription<bluerov_interfaces::msg::ActuatorInput>(
  //     input_topic_name_, rclcpp::QoS(1).deadline(deadline_duration_),
  //     on_input_message, input_subscription_options);
}

void BlueROVActuatorDriverNode::create_cam_tilt_subscription() {
  auto on_tilt_msg =
      [this](const bluerov_interfaces::msg::CameraTilt::SharedPtr msg) {
        auto tilt_deg = msg->tilt_deg;
        bluerov_actuator_driver_.set_aux_pwm_tilt(tilt_deg);
      };
  cam_tilt_sub_ =
      this->create_subscription<bluerov_interfaces::msg::CameraTilt>(
          cam_tilt_topic_name_, rclcpp::QoS(1), on_tilt_msg);
}

void BlueROVActuatorDriverNode::create_light_brightness_subscription() {
  auto on_brightness_msg =
      [this](const bluerov_interfaces::msg::LightBrightness::SharedPtr msg) {
        auto brightness = msg->brightness;
        bluerov_actuator_driver_.set_aux_pwm_light(brightness);
      };
  brightness_sub_ =
      this->create_subscription<bluerov_interfaces::msg::LightBrightness>(
          brightness_topic_name_, rclcpp::QoS(1), on_brightness_msg);
}

void BlueROVActuatorDriverNode::create_joy_subscription() {
  auto on_joy_msg = [this](
                        const sensor_msgs::msg::Joy::SharedPtr last_joy_msg) {
    // received_joy_msg_ptr_.set(std::move(msg));
    if (last_joy_msg->buttons[2] == 1 && bluerov_actuator_driver_.is_running_) {
      RCLCPP_INFO(get_logger(), "Disabling controller input..");
      bluerov_actuator_driver_.is_running_ = false;
      change_pid_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    if (last_joy_msg->buttons[3] == 1 &&
        !bluerov_actuator_driver_.is_running_) {
      RCLCPP_INFO(get_logger(), "Enabling controller input..");
      bluerov_actuator_driver_.is_running_ = true;
      // change_pid_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    if (last_joy_msg->axes[7] == 1 && !last_aux_inc_) {
      bluerov_actuator_driver_.increment_aux_pwm(-10, 0);
      // bluerov_actuator_driver_.send_pwm();
      bluerov_actuator_driver_.set_aux_pwm();
      last_aux_inc_ = true;
      RCLCPP_INFO_STREAM(
          get_logger(),
          "Camera PWM: " << bluerov_actuator_driver_.get_aux_pwm()[0]);
    } else if (last_joy_msg->axes[7] == -1 && !last_aux_inc_) {
      bluerov_actuator_driver_.increment_aux_pwm(10, 0);
      // bluerov_actuator_driver_.send_pwm();
      bluerov_actuator_driver_.set_aux_pwm();
      RCLCPP_INFO_STREAM(
          get_logger(),
          "Camera PWM: " << bluerov_actuator_driver_.get_aux_pwm()[0]);

      last_aux_inc_ = true;
    } else if (last_joy_msg->axes[7] == 0 && last_aux_inc_) {
      last_aux_inc_ = 0;
    }

    if (last_joy_msg->axes[6] == 1) {
      bluerov_actuator_driver_.increment_aux_pwm(0, 10);
      // bluerov_actuator_driver_.send_pwm();
      bluerov_actuator_driver_.set_aux_pwm();
    } else if (last_joy_msg->axes[6] == -1) {
      bluerov_actuator_driver_.increment_aux_pwm(0, -10);
      // bluerov_actuator_driver_.send_pwm();
      bluerov_actuator_driver_.set_aux_pwm();
    }

    // if (msg->buttons[2] == 1 && bluerov_actuator_driver_.is_running_) {
    //   RCLCPP_INFO(get_logger(), "Disabling controller input..");
    //   bluerov_actuator_driver_.is_running_ = false;
    // }
    // if (msg->buttons[3] == 1 && !bluerov_actuator_driver_.is_running_) {
    //   RCLCPP_INFO(get_logger(), "Enabling controller input..");
    //   bluerov_actuator_driver_.is_running_ = true;
    // }
  };
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_name_, rclcpp::QoS(1), on_joy_msg);
}

void BlueROVActuatorDriverNode::create_timer_callback() {
  double voltage_acc = 0;
  int counter = 0;
  int decimation_factor = 10;

  auto timer_callback = [&, this]() {
    auto [voltage, current] = bluerov_actuator_driver_.get_battery_status();
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Battery voltage: " << voltage
    //                                        << " Battery current: " <<
    //                                        current);

    if (voltage < 5.0) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Battery voltage: " << voltage << " is below 5.0V!");
      // hack, the lambda cannot capture the value of the integers (counter
      // and decimation factor), but the first measurements are always well
      // below 5V
      counter = 0;
      decimation_factor = 10;
      return;
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "Counter: " << counter << "
    // Voltage_acc: " << voltage_acc << " Dec factor: " << decimation_factor);

    voltage_acc += voltage;
    counter += 1;

    if (counter % decimation_factor == 0) {
      // RCLCPP_INFO_STREAM(get_logger(), "Battery voltage: "
      //                                      << voltage
      //                                      << " Battery current: " <<
      //                                      current);

      sensor_msgs::msg::BatteryState msg;

      msg.voltage = voltage_acc / decimation_factor;

      RCLCPP_INFO_STREAM(get_logger(), "Battery avg voltage: "
                                           << msg.voltage
                                           << " Battery current: " << current);

      battery_voltage_pub_->publish(msg);

      voltage_acc = 0.0;
    }
  };
  timer_ = this->create_wall_timer(update_period_, timer_callback);
  // cancel immediately to prevent triggering it in this state
  timer_->cancel();
}

bool BlueROVActuatorDriverNode::change_pid_state(
    std::uint8_t transition, std::chrono::seconds time_out) {
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  if (!client_change_pid_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                 client_change_pid_state_->get_service_name());
    return false;
  }

  // We send the request with the transition we want to invoke.
  auto future_result =
      client_change_pid_state_->async_send_request(request).future.share();

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Server time out while getting current state for node");
    return false;
  }

  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.",
                static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(get_logger(), "Failed to trigger transition %u",
                static_cast<unsigned int>(transition));
    return false;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BlueROVActuatorDriverNode::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Configuring");
  bluerov_actuator_driver_.is_running_ = false;
  // bluerov_actuator_driver_.set_pwm();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BlueROVActuatorDriverNode::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Activating");
  //   observer_pub_->on_activate();
  battery_voltage_pub_->on_activate();
  pwm_pub_->on_activate();
  timer_->reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BlueROVActuatorDriverNode::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  timer_->cancel();
  bluerov_actuator_driver_.is_running_ = false;
  battery_voltage_pub_->on_deactivate();
  pwm_pub_->on_deactivate();
  //   observer_pub_->on_deactivate();
  // log the status to introspect the result
  // log_observer_state();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BlueROVActuatorDriverNode::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BlueROVActuatorDriverNode::on_shutdown(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  bluerov_actuator_driver_.is_running_ = false;
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace bluerov_actuator_driver
#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    bluerov_actuator_driver::BlueROVActuatorDriverNode)
