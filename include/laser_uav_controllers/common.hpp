#ifndef LASER_UAV_CONTROLLERS_COMMON_HPP
#define LASER_UAV_CONTROLLERS_COMMON_HPP

#include <iostream>
#include <Eigen/Dense>

namespace laser_uav_controllers
{
#define GRAVITY 9.80665

/* quadrotor_t //{ */
struct quadrotor_t
{
  double          mass;
  int             n_motors;
  Eigen::MatrixXd G1;
  Eigen::MatrixXd G2;
  Eigen::Matrix3d inertia_matrix;
  double          motor_inertia;
  double          c_thrust;
  Eigen::Vector3d drag;
  double          thrust_min;
  double          thrust_max;
  double          total_thrust_max;
  double          motor_curve_a;
  double          motor_curve_b;
};
//}

double thrustToThrotle(double a, double b, double thrust);
double thrustToThrotle(double a, double b, double thrust, double thrust_max, double thrust_min);
}  // namespace laser_uav_controllers

#endif
