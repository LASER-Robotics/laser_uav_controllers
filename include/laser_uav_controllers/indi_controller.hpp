#ifndef LASER_UAV_CONTROLLERS__INDI_CONTROLLER_HPP
#define LASER_UAV_CONTROLLERS__INDI_CONTROLLER_HPP

#include <laser_uav_controllers/common.hpp>

namespace laser_uav_controllers
{
class IndiController {
public:
  IndiController();
  IndiController(quadrotor_t quadrotor_params);

  Eigen::VectorXd getCorrection(Eigen::Vector3d& angular_acceleration_estimated, Eigen::VectorXd& motor_speed_estimated, Eigen::VectorXd& individual_thrust,
                                Eigen::Vector3d& angular_speed /*, double dt*/);

private:
  double dt = 0.004;
  double thrust_coefficient_;

  Eigen::MatrixXd G1_;
  Eigen::MatrixXd G2_;
  Eigen::Matrix3d inertia_matrix_;

  Eigen::VectorXd motor_speed_estimated_prev_;

  Eigen::VectorXd u_;
};
}  // namespace laser_uav_controllers

#endif
