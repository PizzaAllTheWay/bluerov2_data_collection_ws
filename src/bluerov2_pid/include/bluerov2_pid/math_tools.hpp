#include <Eigen/Dense>

namespace math_tools
{

 inline Eigen::Matrix3d skew(const Eigen::Vector3d& x)
{
  Eigen::Matrix3d x_skew;
  x_skew << 0.0, -x(2), x(1),
            x(2), 0.0, -x(0),
            -x(1), x(0), 0.0;
  return x_skew;
}

inline Eigen::Matrix<double, 6, 6> smallAd(const Eigen::Vector<double, 6>& x)
{
  Eigen::Matrix<double, 6, 6> x_smallAd;
  x_smallAd << skew(x.tail(3)), x.head(3),
               Eigen::MatrixXd::Zero(1,4); 

  return x_smallAd;
}


inline Eigen::Matrix<double, 4, 3> quat_transform_matrix(const Eigen::Quaterniond& quat)
{
  Eigen::Matrix<double, 4, 3> transf;
//   transf << 0.5 * -Eigen::RowVector3d(quat.vec()),
            // 0.5 * ( quat.w() * Eigen::Matrix3d::Identity() + skew(quat.vec()) );

  transf.row(0) = 0.5 * -Eigen::RowVector3d(quat.vec());
  transf.middleRows(1, 3) = 0.5 * ( quat.w() * Eigen::Matrix3d::Identity() + skew(quat.vec()) );
  return transf;
}

inline double distance_quat(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
  return (1 - (q1.w() * q2.w() + q1.vec().transpose() * q2.vec()));
}

inline Eigen::Matrix<double, 6, 6> Ad_se3(const Eigen::Matrix3d & Rot,
                                          const Eigen::Vector3d & pos)
{
    Eigen::Matrix<double, 6, 6> Ad = Eigen::MatrixXd::Zero(6,6);
    Ad.topLeftCorner(3,3) = Rot;
    Ad.topRightCorner(3,3) = skew(pos) * Rot;
    Ad.bottomRightCorner(3,3) = Rot;
    return Ad;
}

inline Eigen::Matrix<double, 6, 6> Ad_se3_inv(const Eigen::Matrix3d & Rot,
                                              const Eigen::Vector3d & pos)
{
    Eigen::Matrix<double, 6, 6> Ad_inv = Eigen::MatrixXd::Zero(6,6);
    Ad_inv.topLeftCorner(3,3) = Rot.transpose();
    Ad_inv.topRightCorner(3,3) = - Rot.transpose() * skew(pos);
    Ad_inv.bottomRightCorner(3,3) = Rot.transpose();
    return Ad_inv;
}


template <typename T> T constrain(T val, T min_val, T max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

// template <typename T> inline int sgn(T val) {
//     return (T(0) < val) - (val < T(0));
// }

}