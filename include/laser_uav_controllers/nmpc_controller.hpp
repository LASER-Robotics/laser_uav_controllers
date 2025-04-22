#ifndef LASER_UAV_CONTROLLERS__NMPC_CONTROLLER_HPP
#define LASER_UAV_CONTROLLERS__NMPC_CONTROLLER_HPP

#include <Eigen/Dense>
#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>
#include <mpc/Types.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace laser_uav_controllers
{

constexpr int _nx_  = 13;
constexpr int _ny_  = 13;
constexpr int _nu_  = 4;
constexpr int _nxh_ = 10;
constexpr int _nuh_ = 4;
constexpr int _ni_  = 0;
constexpr int _ne_  = 0;
#define _ts_ 0.1
#define GRAVITY 9.8
#define Ixx_ 0.4953
#define Iyy_ 0.4953
#define Izz_ 0.3413
#define mass_ 2.76
#define cq_ 4.39e-08
#define ct_ 7.97e-08
#define l_ 0.25455
#define beta_ 0.4

#define X 0
#define Y 1
#define Z 2
#define QW 3
#define QX 4
#define QY 5
#define QZ 6
#define VX 7
#define VY 8
#define VZ 9
#define OX 10
#define OY 11
#define OZ 12

class NmpcController {
public:
  /* NmpcController(int n_horizon, double mass, Eigen::VectorXd weights, double wn_factor); */
  NmpcController();
  /* NmpcController(int a); */

  /* laser_msgs::msg::ThrustAndTorque getCorrection(geometry::msg::Pose reference, const nav_msgs::msg::Odometry &msg); */
  void setCostFunction();
  void setUavDynamics();
  void getCorrection(geometry_msgs::msg::Pose ref, const nav_msgs::msg::Odometry msg);

private:
  mpc::cvec<_ny_>                                        reference;
  mpc::NLMPC<_nx_, _nu_, _ny_, _nxh_, _nuh_, _ni_, _ne_> nlmpc_solver_;
  /* mpc::NLMPC<> nlmpc_solver_; */

  Eigen::Matrix<double, 13, 13> Q;
  /* double mass_; */
  /* double ct_; */
  double uss_;
};
}  // namespace laser_uav_controllers

#endif
