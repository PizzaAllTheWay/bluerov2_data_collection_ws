#include <cmath>
#include <utility>
#include <algorithm>
#include "bluerov2_pid/bluerov2_pid.hpp"
// #include "math_tools.hpp"

#include <iostream>
namespace bluerov2_pid
{

PIDController::PIDController(
    std::vector<double> Kp,
    std::vector<double> Kd,
    std::vector<double> Ki,
    std::chrono::milliseconds update_period,
    std::vector<double> xi_lb,
    std::vector<double> xi_ub,
    double delta,
    double attitude_gain
  )
  : integral_limits_lower_{Eigen::Map<Eigen::Vector<double, 6>>(xi_lb.data(), 6)},
    integral_limits_upper_{Eigen::Map<Eigen::Vector<double, 6>>(xi_ub.data(), 6)},
    Kp_{Eigen::Map<Eigen::Vector3d>(Kp.data(), 3).asDiagonal()},
    Kd_{Eigen::Map<Vector6d>(Kd.data(), 6).asDiagonal()},
    Ki_{Eigen::Map<Vector6d>(Ki.data(), 6).asDiagonal()},
    bias_hat_{Eigen::VectorXd::Zero(6)},
    pos_{Eigen::Vector3d::Zero()},
    quat_{Eigen::Quaterniond::Identity()},
    nu_{Vector6d::Zero()},
    pos_ref_{Eigen::Vector3d::Zero()},
    quat_ref_{Eigen::Quaterniond::Identity()},
    nu_ref_{Vector6d::Zero()},
    nu_dot_ref_{Vector6d::Zero()},
    control_inputs_{Eigen::Matrix<double, 8, 1>::Zero(8, 1)},
    delta_{delta},
    k_{attitude_gain},
    B_{Eigen::Matrix<double, 6, 8>::Zero(6, 8)}
  {
    dt_ = update_period.count() / (1000.0);
    LENGTHS_THRUSTERS << 0.156, 0.156, -0.156, -0.156, 0.12, 0.12, -0.12, -0.12,
                      0.111, -0.111, 0.111, -0.111, 0.218, -0.218, 0.218, -0.218,
                      0.085, 0.085, 0.085, 0.085, 0, 0, 0, 0; 

    Eigen::Vector3d e_1 = {1, 0, 0};
    Eigen::Vector3d e_3 = {0, 0, 1};

    for (int i = 0; i < 4; i++)
    {
      Eigen::Vector3d thrust_dir = MIXER[i] * Sophus::SO3d::rotZ(kPi / 180.0 * ALPHA[i]).matrix() * e_1;
      B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i));
    }
    for (int i = 4; i < 8; i++)
    {
      Eigen::Vector3d thrust_dir = - MIXER[i] * e_3;
      B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i)); 
    }

  }

void PIDController::set_horizontal_thruster_angle_deg(unsigned int angle_deg)
  {
    ALPHA[0] = -angle_deg;
    ALPHA[1] =  angle_deg;
    ALPHA[2] = -180 + angle_deg ;
    ALPHA[3] =  180 - angle_deg;
  }

void PIDController::update()
{
  Sophus::SO3d R_d = Sophus::SO3d(quat_ref_);
  Sophus::SO3d R = Sophus::SO3d(quat_);
  Sophus::SO3d R_e = R_d.inverse() * R;
  Eigen::Vector3d p_e = R_d.matrix().transpose() * (pos_ - pos_ref_);
  Sophus::SE3d g_e(R_e.matrix(), p_e);

  // jump map
  Eigen::Matrix<double, 4, 1> quat_e = R_e.unit_quaternion().coeffs(); // Sophus: q = (imag0, imag1, imag2, real)
  if (q_ * quat_e(3) <= -delta_) {
    q_ = -q_;
  }

  Eigen::Vector3d dV_ang = k_ * q_ * quat_e.head(3);

  // natural error
  Vector6d nu_r = g_e.inverse().Adj() *nu_ref_;
  Vector6d nu_e = nu_ - nu_r;
  // Vector6d nu_r_dot = g_e.inverse().Adj() * nu_dot_ref_ 
              // - math_tools::smallAd(nu_e) * g_e.inverse().Adj() * nu_ref_;

  Vector6d dV;
  dV << R_e.matrix().transpose() * Kp_ * p_e, dV_ang;


  // compute control input
  // control_inputs_ = B_.completeOrthogonalDecomposition().solve(
  //   - dV - Kd_ * nu_e - Ki_ * bias_hat_ );
  Eigen::Vector<double, 6> grav;
  grav << R.matrix().transpose() * Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero();

  control_inputs_ = B_.completeOrthogonalDecomposition().solve(
    grav - dV - Kd_ * nu_e - Ki_ * bias_hat_ );

  // integrate
  // Eigen::Vector<double, 6> bias_hat_dot = dV + Kd_ * nu_e;
  Eigen::Vector<double, 6> bias_hat_dot = dV;

  bias_hat_ = bias_hat_ + dt_ * bias_hat_dot;


  // Eigen::Vector<double, 6> integral_limits
  for (unsigned int i = 0; i < 6; i++) {
    bias_hat_(i) = std::clamp(bias_hat_(i), integral_limits_lower_(i) / Ki_.diagonal()[i], integral_limits_upper_(i) / Ki_.diagonal()[i]);
  }

}

} // namespace bluerov2_pid
