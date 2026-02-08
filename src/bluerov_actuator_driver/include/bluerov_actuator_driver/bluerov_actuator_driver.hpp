#ifndef BLUEROV_ACTUATOR_DRIVER__BLUEROV_ACTUATOR_DRIVER_HPP_
#define BLUEROV_ACTUATOR_DRIVER__BLUEROV_ACTUATOR_DRIVER_HPP_

#include "bluerov_actuator_driver/pixhawk.hpp"
#include "bluerov_actuator_driver/visibility_control.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

constexpr uint32_t THRUSTER_MAX_PWM = 1900;
constexpr uint32_t THRUSTER_MIN_PWM = 1100;
constexpr unsigned int CAMERA_MAX_PWM = 2000;
constexpr unsigned int CAMERA_MIN_PWM = 1200;
constexpr unsigned int LIGHTS_MAX_PWM = 1900;
constexpr unsigned int LIGHTS_MIN_PWM = 1100;
constexpr uint32_t THRUSTER_ZERO_PWM = 1500;
constexpr unsigned int CAMERA_ZERO_PWM = 1650;
constexpr unsigned int LIGHTS_ZERO_PWM = 1100;


namespace bluerov_actuator_driver
{

class BLUEROV_ACTUATOR_DRIVER_PUBLIC BlueROVActuatorDriver {
public:
  BlueROVActuatorDriver();
  ~BlueROVActuatorDriver() = default;

  void send_pwm();

  std::vector<uint32_t> get_aux_pwm();
  std::vector<uint32_t> get_thruster_pwm();

  void increment_aux_pwm(const uint32_t delta_pwm_cam, const uint32_t delta_pwm_light);
  void set_aux_pwm();
  void set_aux_pwm_tilt(const double tilt_deg);
  void set_aux_pwm_light(const double brightness);

  void process_control_inputs(const std::vector<double> inputs);

  std::tuple<float, float> get_battery_status();

  double clip(double n, double lower, double upper);
  
  bool is_running_;
  
private:
  // uint32_t pwm_[10];
  std::vector<uint32_t> thruster_pwm_;
  std::vector<double> thruster_unsat_pwm_;
  std::vector<uint32_t> aux_pwm_;

  Pixhawk px6_;
};
}
#endif // BlueROV_ACTUATOR_DRIVER__BlueROV_ACTUATOR_DRIVER_HPP_
