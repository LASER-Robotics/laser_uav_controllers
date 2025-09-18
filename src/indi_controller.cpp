#include <laser_uav_controllers/indi_controller.hpp>

namespace laser_uav_controllers
{
/* IndiController() {default} //{ */
IndiController::IndiController() {
}
//}

/* IndiController() //{ */
IndiController::IndiController(quadrotor_t quadrotor_params) {
  u_                          = Eigen::VectorXd(quadrotor_params.n_motors);
  motor_speed_estimated_prev_ = Eigen::VectorXd(quadrotor_params.n_motors);

  thrust_coefficient_ = quadrotor_params.c_thrust;

  G1_ = quadrotor_params.G1;
  G2_ = quadrotor_params.G2;

  inertia_matrix_ = quadrotor_params.inertia_matrix;
}
//}

///* getCorrection() //{ */
Eigen::VectorXd IndiController::getCorrection(Eigen::Vector3d& angular_acceleration_estimated, Eigen::VectorXd& motor_speed_estimated,
                                              Eigen::VectorXd& individual_thrust, Eigen::Vector3d& angular_speed /*, double dt*/) {
  Eigen::Vector3d angular_acceleration_desired =
      inertia_matrix_.inverse() * ((G1_.bottomRows(3) * individual_thrust) - angular_speed.cross(inertia_matrix_ * angular_speed));

  Eigen::Vector3d torque_estimated = (G1_.bottomRows(3) * (thrust_coefficient_ * motor_speed_estimated.cwiseProduct(motor_speed_estimated))) +
                                     ((1.0 / dt) * (G2_.bottomRows(3) * (thrust_coefficient_ * (motor_speed_estimated - motor_speed_estimated_prev_))));
  motor_speed_estimated_prev_ = motor_speed_estimated;

  Eigen::Vector3d torque_desired = torque_estimated + (inertia_matrix_ * (angular_acceleration_desired - angular_acceleration_estimated));

  Eigen::Vector4d desired_command;
  desired_command << individual_thrust.sum(), torque_desired;

  u_ = ((G1_ * thrust_coefficient_).inverse() * desired_command).cwiseMax(0).cwiseSqrt();
  u_ = u_.cwiseProduct(u_) * thrust_coefficient_;

  return u_;
}
//}
}  // namespace laser_uav_controllers
