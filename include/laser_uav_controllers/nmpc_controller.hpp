#ifndef LASER_UAV_CONTROLLERS__NMPC_CONTROLLER_HPP
#define LASER_UAV_CONTROLLERS__NMPC_CONTROLLER_HPP

#include <laser_uav_controllers/common.hpp>

#include "acados/utils/math.h"
#include <acados_c/ocp_nlp_interface.h>
#include <acados_c/external_function_interface.h>
#include <acados/ocp_nlp/ocp_nlp_constraints_bgh.h>
#include <acados/ocp_nlp/ocp_nlp_cost_ls.h>

#include <blasfeo/include/blasfeo_d_aux.h>
#include <blasfeo/include/blasfeo_d_aux_ext_dep.h>

#include <multirotor_ode_model/multirotor_ode_model.h>
#include <acados_solver_multirotor_ode.h>

#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/reference_state.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>

namespace laser_uav_controllers
{
#define NX MULTIROTOR_ODE_NX
#define NU MULTIROTOR_ODE_NU
#define NY MULTIROTOR_ODE_NY
#define NYN MULTIROTOR_ODE_NYN
#define NP MULTIROTOR_ODE_NP

/* acados_t //{ */
struct acados_t
{
  std::string         nmpc_mode;
  int                 N;
  double              dt;
  std::vector<double> Q;
  double              R;
};
//}

class NmpcController {
public:
  NmpcController();
  NmpcController(multirotor_t multirotor_params, acados_t acados_params);

  std::pair<Eigen::Vector3d, Eigen::VectorXd> getCorrection(laser_msgs::msg::ReferenceState reference, const nav_msgs::msg::Odometry msg);
  std::pair<Eigen::Vector3d, Eigen::VectorXd> getCorrection(std::vector<laser_msgs::msg::ReferenceState> trajectory, const nav_msgs::msg::Odometry msg);

  std::vector<double> getLastIndividualThrust();
  void                setMass(double mass);
  double getOcpElapsedTime();

private:
  void setInitState();
  void setInitSolution();
  void setTrajectory(std::vector<laser_msgs::msg::ReferenceState> trajectory);
  bool ocpSolver();
  void printStatistics();
  void getFirstControlInput();
  void getFirstComputedStates();
  void printAllComputedStates();
  void printAllControlInputs();

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
    w4 = 3,
    w5 = 4,
    w6 = 5,
    w7 = 6,
    w8 = 7
  };
  //}

  /* params_e //{ */
  enum params_e
  {
    mass         = 0,
    G1           = 1,
    inertia      = 33,
    drag         = 36,
    qw_reference = 39,
    qx_reference = 40,
    qy_reference = 41,
    qz_reference = 42
  };
  //}

  int N;

  double x0_[NX];
  double yref_[NX];

  double u0_[NU];
  double x1_[NX];

  int    n_motors_;
  double hover_thrust_;
  double motor_curve_a_;
  double motor_curve_b_;

  double parameters_[NP] = {0};

  multirotor_ode_solver_capsule *acados_ocp_capsule;

  bool angular_rates_and_thrust_mode_;
};
}  // namespace laser_uav_controllers

#endif
