#ifndef LASER_UAV_CONTROLLERS__NMPC_CONTROLLER_HPP
#define LASER_UAV_CONTROLLERS__NMPC_CONTROLLER_HPP

#include <iostream>

#include "acados/utils/math.h"
#include <acados_c/ocp_nlp_interface.h>
#include <acados_c/external_function_interface.h>
#include <acados/ocp_nlp/ocp_nlp_constraints_bgh.h>
#include <acados/ocp_nlp/ocp_nlp_cost_ls.h>

#include <blasfeo/include/blasfeo_d_aux.h>
#include <blasfeo/include/blasfeo_d_aux_ext_dep.h>

#include <quadrotor_ode_model/quadrotor_ode_model.h>
#include <acados_solver_quadrotor_ode.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>

namespace laser_uav_controllers
{

#define N QUADROTOR_ODE_N
#define NX QUADROTOR_ODE_NX
#define NU QUADROTOR_ODE_NU
#define NY QUADROTOR_ODE_NY
#define NYN QUADROTOR_ODE_NYN
#define NP QUADROTOR_ODE_NP

#define GRAVITY 9.80665

/* quadrotor_t //{ */
struct quadrotor_t
{
  double mass;
  double motor_position_0[2];
  double motor_position_1[2];
  double motor_position_2[2];
  double motor_position_3[2];
  double inertia_x;
  double inertia_y;
  double inertia_z;
  double c_tau;
  double drag_x;
  double drag_y;
  double drag_z;
  double thrust_min;
  double thrust_max;
  double total_thrust_max;
  double motor_curve_a;
  double motor_curve_b;
};
//}

/* acados_t //{ */
struct acados_t
{
  std::vector<double> Q;
  double              R;
};
//}

class NmpcController {
public:
  NmpcController();
  NmpcController(quadrotor_t quadrotor_params, acados_t acados_params);

  laser_msgs::msg::AttitudeRatesAndThrust getCorrection(geometry_msgs::msg::Pose reference, const nav_msgs::msg::Odometry msg);

private:
  void   setInitState();
  void   setInitSolution();
  void   setReference();
  bool   ocpSolver();
  void   printStatistics();
  void   getFirstControlInput();
  void   getFirstComputedStates();
  void   printAllComputedStates();
  void   printAllControlInputs();
  double thrustToThrotle();

  /* states_e //{ */
  enum states_e
  {
    x   = 0,
    y   = 1,
    z   = 2,
    qw  = 3,
    qx  = 4,
    qy  = 5,
    qz  = 6,
    vbx = 7,
    vby = 8,
    vbz = 9,
    wx  = 10,
    wy  = 11,
    wz  = 12
  };
  //}

  /* control_input_e //{ */
  enum control_input_e
  {
    w1 = 0,
    w2 = 1,
    w3 = 2,
    w4 = 3
  };
  //}

  /* params_e //{ */
  enum params_e
  {
    mass               = 0,
    motor_position_0_x = 1,
    motor_position_0_y = 2,
    motor_position_1_x = 3,
    motor_position_1_y = 4,
    motor_position_2_x = 5,
    motor_position_2_y = 6,
    motor_position_3_x = 7,
    motor_position_3_y = 8,
    inertia_x          = 9,
    inertia_y          = 10,
    inertia_z          = 11,
    c_tau              = 12,
    drag_x             = 13,
    drag_y             = 14,
    drag_z             = 15,
    qw_reference       = 16,
    qx_reference       = 17,
    qy_reference       = 18,
    qz_reference       = 19
  };
  //}

  double x0_[NX];
  double yref_[NYN];

  double u0_[NU];
  double x1_[NX];

  double hover_thrust_;
  double motor_curve_a_;
  double motor_curve_b_;

  double parameters_[NP];

  quadrotor_ode_solver_capsule *acados_ocp_capsule;
};
}  // namespace laser_uav_controllers

#endif
