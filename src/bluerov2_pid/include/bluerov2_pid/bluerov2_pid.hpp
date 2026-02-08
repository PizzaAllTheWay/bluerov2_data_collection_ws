#ifndef HYSTERETIC_CONTROLLER__HYSTERETIC_CONTROLLER_HPP_
#define HYSTERETIC_CONTROLLER__HYSTERETIC_CONTROLLER_HPP_

#include <vector>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include "sophus/geometry.hpp"
// #include "manif/manif.h"
#include "math_tools.hpp"
#include <algorithm>
// #include "bluerov2_pid/runge_kutta.hpp"

namespace bluerov2_pid
{

// using namespace Eigen;
const double kPi = Sophus::Constants<double>::pi();
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;


class PIDController
{
public:
  static constexpr unsigned int INPUT_DIM = 8;

  Eigen::Matrix<double, 3, 8> LENGTHS_THRUSTERS;
  std::vector<int> MIXER = {1, 1, -1, -1, 1, -1, -1, 1};
  std::vector<int> ALPHA = {-45, 45, -135, 135};


  PIDController(
    std::vector<double> Kp,
    std::vector<double> Kd,
    std::vector<double> Ki,
    std::chrono::milliseconds update_period,
    std::vector<double> xi_lb,
    std::vector<double> xi_ub,
    double delta,
    double k_global
  );

  void set_horizontal_thruster_angle_deg(unsigned int angle_deg);

  void update();
  
  const Vector8d get_control_input()
  {
    return control_inputs_;
  }

  void set_gains(
    std::vector<double> Kp,
    std::vector<double> Kd,
    std::vector<double> Ki,
    double attitude_gain)
  {
    Kp_ = Eigen::Map<Eigen::Vector3d>(Kp.data(), 3).asDiagonal();
    Kd_ = Eigen::Map<Vector6d>(Kd.data(), 6).asDiagonal();
    Ki_ = Eigen::Map<Vector6d>(Ki.data(), 6).asDiagonal();
    k_ = attitude_gain;
  }

  void set_integral_limits(
    std::vector<double> lower_bounds,
    std::vector<double> upper_bounds
  )
  {
    integral_limits_lower_ = Eigen::Map<Eigen::Vector<double, 6>>(lower_bounds.data(), 6);
    integral_limits_upper_ = Eigen::Map<Eigen::Vector<double, 6>>(upper_bounds.data(), 6);
  }


  const Eigen::Vector<double, 6> get_integral_state()
  {
    return bias_hat_;
  }

  int get_switching_state()
  {
    return q_;
  }

  void set_measurements(const Eigen::Vector3d & pos, const Eigen::Quaterniond & quat, const Vector6d & nu)
  {
    pos_ << pos;
    quat_ = quat;
    nu_ << nu;
  }

  void set_references(
    const Eigen::Vector3d & pos_ref, const Eigen::Quaterniond & quat_ref, const Vector6d & nu_ref,
    const Vector6d & nu_dot_ref)
  {
    pos_ref_ << pos_ref;
    quat_ref_ = quat_ref;
    nu_ref_ << nu_ref;
    nu_dot_ref_ << nu_dot_ref;
  }

  void clear_integral_state() {
    bias_hat_ = Vector6d::Zero();
  }


  Eigen::Vector<double, 6> integral_limits_lower_;
  Eigen::Vector<double, 6> integral_limits_upper_;

private:
  // Controller gains
  Matrix3d Kp_;
  Eigen::DiagonalMatrix<double, 6> Kd_;
  Eigen::DiagonalMatrix<double, 6> Ki_;

  // Controller states
  Eigen::Vector<double, 6> bias_hat_;

  Eigen::Vector3d pos_;
  Eigen::Quaterniond quat_;
  Vector6d nu_;
  Eigen::Vector3d pos_ref_;
  Eigen::Quaterniond quat_ref_;
  Vector6d nu_ref_;
  Vector6d nu_dot_ref_;
  

  Eigen::Matrix<double, 8, 1> control_inputs_;

  int q_{1};   // discrete switching state

  double dt_;
  double delta_;
  double k_;

  Eigen::Matrix<double, 6, 8> B_;

};

}

#endif // HYSTERETIC_CONTROLLER__HYSTERETIC_CONTROLLER_HPP_