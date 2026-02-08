#include "bluerov_actuator_driver/bluerov_actuator_driver.hpp"
#include <bits/stdint-uintn.h>

namespace bluerov_actuator_driver
{
BlueROVActuatorDriver::BlueROVActuatorDriver()
    : is_running_{false},
      thruster_pwm_{THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM},
      thruster_unsat_pwm_{THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM},
      aux_pwm_{CAMERA_ZERO_PWM, LIGHTS_ZERO_PWM},
      px6_()
{
  send_pwm();
}

void BlueROVActuatorDriver::send_pwm()
{
  // uint32_t pwm[10] = {thruster_pwm_[0], thruster_pwm_[1], thruster_pwm_[2], thruster_pwm_[3], thruster_pwm_[4], thruster_pwm_[5], thruster_pwm_[6], thruster_pwm_[7], aux_pwm_[0], aux_pwm_[1]};
  uint32_t pwm[8] = {thruster_pwm_[0], thruster_pwm_[1], thruster_pwm_[2], thruster_pwm_[3], thruster_pwm_[4], thruster_pwm_[5], thruster_pwm_[6], thruster_pwm_[7]};
  px6_.set_pwm_us(pwm, sizeof(pwm));
}

void BlueROVActuatorDriver::increment_aux_pwm(const uint32_t delta_pwm_cam, const uint32_t delta_pwm_light)
{
  aux_pwm_[0] += delta_pwm_cam;
  aux_pwm_[1] += delta_pwm_light;
}


void BlueROVActuatorDriver::set_aux_pwm()
{
  // camera PWM
  aux_pwm_[0] = clip(aux_pwm_[0], CAMERA_MIN_PWM, CAMERA_MAX_PWM);
  // light PWM
  aux_pwm_[1] = clip(aux_pwm_[1], LIGHTS_MIN_PWM, LIGHTS_MAX_PWM);
  uint32_t aux_pwm[4] = {aux_pwm_[0], aux_pwm_[1], 0, 0};
  px6_.set_io_pwm_us(aux_pwm, sizeof(aux_pwm));
}

void BlueROVActuatorDriver::set_aux_pwm_tilt(const double tilt_deg)
{
  aux_pwm_[0] = 1600 + 800.0/90.0 * tilt_deg;
  // camera PWM
  aux_pwm_[0] = clip(aux_pwm_[0], CAMERA_MIN_PWM, CAMERA_MAX_PWM);
  // light PWM
  aux_pwm_[1] = clip(aux_pwm_[1], LIGHTS_MIN_PWM, LIGHTS_MAX_PWM);
  uint32_t aux_pwm[4] = {aux_pwm_[0], aux_pwm_[1], 0, 0};
  px6_.set_io_pwm_us(aux_pwm, sizeof(aux_pwm));
}

void BlueROVActuatorDriver::set_aux_pwm_light(const double brightness)
{
  // Ensure brightness is between 0 and 1
  double brightness_clipped = clip(brightness, 0.0, 1.0);
  // Convert brightness to PWM
  aux_pwm_[1] = static_cast<uint32_t>(LIGHTS_MIN_PWM + brightness_clipped * (LIGHTS_MAX_PWM - LIGHTS_MIN_PWM));
  // light PWM
  uint32_t aux_pwm[4] = {aux_pwm_[0], aux_pwm_[1], 0, 0};
  px6_.set_io_pwm_us(aux_pwm, sizeof(aux_pwm));
}

void BlueROVActuatorDriver::process_control_inputs(const std::vector<double> inputs)
{
  if (is_running_ && inputs.size() == 8) {
    for (int i = 0; i < 8; i++)
    {
      if (inputs[i] > 0)
      {
        thruster_unsat_pwm_[i] = 1.8129 * pow(inputs[i] / 9.81, 3) - 18.206 * pow(inputs[i] / 9.81, 2) + 120.5 * (inputs[i] / 9.81) + 1525;
      }
      else if (inputs[i] < 0)
      {
        thruster_unsat_pwm_[i] = 4.3035 * pow(inputs[i] / 9.81, 3) + 32.806 * pow(inputs[i] / 9.81, 2) + 152.24 * (inputs[i] / 9.81) + 1475;
      }
      else
      {
        thruster_unsat_pwm_[i] = 1500;
      }
      // pwm[i] = clip(pwm[i], THRUSTER_MIN_PWM, THRUSTER_MAX_PWM);
      // pwm_[i] = (uint32_t)pwm[i];
      thruster_pwm_[i] = (uint32_t)clip(thruster_unsat_pwm_[i], THRUSTER_MIN_PWM, THRUSTER_MAX_PWM);
    }
  } 
  else 
  {
    thruster_pwm_ = {THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM, THRUSTER_ZERO_PWM};
  }
}


std::tuple<float, float> BlueROVActuatorDriver::get_battery_status()
{
  return px6_.get_battery_status();
}

std::vector<uint32_t> BlueROVActuatorDriver::get_aux_pwm() {
  return aux_pwm_;
}

std::vector<uint32_t> BlueROVActuatorDriver::get_thruster_pwm() {
  return thruster_pwm_;
}

double BlueROVActuatorDriver::clip(double n, double lower, double upper)
{
  return std::max(lower, std::min(n, upper));
}

}
