#include <laser_uav_controllers/nmpc_controller.hpp>

namespace laser_uav_controllers
{
/* NmpcController() {defalut} //{ */
NmpcController::NmpcController() {
}
//}

/* NmpcController() //{ */
NmpcController::NmpcController(quadrotor_t quadrotor_params, acados_t acados_params) {
  std::cout << "creating acados ocp solver" << std::endl;
  acados_ocp_capsule = quadrotor_ode_acados_create_capsule();

  double *dt = new double[N];
  std::fill_n(dt, N, 2.0 / N);
  int status = quadrotor_ode_acados_create_with_discretization(acados_ocp_capsule, N, dt);

  if (status) {
    printf("creation of acados ocp solver returned status %d. Exiting.\n", status);
    exit(1);
  }

  parameters_[params_e::mass]               = quadrotor_params.mass;
  parameters_[params_e::motor_position_0_x] = quadrotor_params.motor_position_0[0];
  parameters_[params_e::motor_position_0_y] = quadrotor_params.motor_position_0[1];
  parameters_[params_e::motor_position_1_x] = quadrotor_params.motor_position_1[0];
  parameters_[params_e::motor_position_1_y] = quadrotor_params.motor_position_1[1];
  parameters_[params_e::motor_position_2_x] = quadrotor_params.motor_position_2[0];
  parameters_[params_e::motor_position_2_y] = quadrotor_params.motor_position_2[1];
  parameters_[params_e::motor_position_3_x] = quadrotor_params.motor_position_3[0];
  parameters_[params_e::motor_position_3_y] = quadrotor_params.motor_position_3[1];
  parameters_[params_e::inertia_x]          = quadrotor_params.inertia_x;
  parameters_[params_e::inertia_y]          = quadrotor_params.inertia_y;
  parameters_[params_e::inertia_z]          = quadrotor_params.inertia_z;
  parameters_[params_e::c_tau]              = quadrotor_params.c_tau;
  parameters_[params_e::drag_x]             = quadrotor_params.drag_x;
  parameters_[params_e::drag_y]             = quadrotor_params.drag_y;
  parameters_[params_e::drag_z]             = quadrotor_params.drag_z;

  // --- Set Nominal Quaternion in Initialize
  parameters_[params_e::qw_reference] = 1;
  parameters_[params_e::qx_reference] = 0;
  parameters_[params_e::qy_reference] = 0;
  parameters_[params_e::qz_reference] = 0;

  motor_curve_a_ = quadrotor_params.motor_curve_a;
  motor_curve_b_ = quadrotor_params.motor_curve_b;

  // --- Calculate Hover Thrust in Newtons
  hover_thrust_ = (parameters_[mass] * GRAVITY) / 4;

  // --- Set Weight Matrix for W
  double W[NY * NY] = {};
  W[(1 + NY) * 0]   = acados_params.Q[0];
  W[(1 + NY) * 1]   = acados_params.Q[0];
  W[(1 + NY) * 2]   = acados_params.Q[1];
  W[(1 + NY) * 3]   = acados_params.Q[2];
  W[(1 + NY) * 4]   = acados_params.Q[2];
  W[(1 + NY) * 5]   = acados_params.Q[3];
  W[(1 + NY) * 6]   = acados_params.Q[4];
  W[(1 + NY) * 7]   = acados_params.Q[4];
  W[(1 + NY) * 8]   = acados_params.Q[4];
  W[(1 + NY) * 9]   = acados_params.Q[5];
  W[(1 + NY) * 10]  = acados_params.Q[5];
  W[(1 + NY) * 11]  = acados_params.Q[5];
  W[(1 + NY) * 12]  = acados_params.R;
  W[(1 + NY) * 13]  = acados_params.R;
  W[(1 + NY) * 14]  = acados_params.R;
  W[(1 + NY) * 15]  = acados_params.R;

  for (int i = 0; i < N; i++) {
    ocp_nlp_cost_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, i, "W", W);
  }

  // --- Set Weight Matrix For W_e
  double W_e[NYN * NYN] = {};
  W_e[(1 + NYN) * 0]    = acados_params.Q[0];
  W_e[(1 + NYN) * 1]    = acados_params.Q[0];
  W_e[(1 + NYN) * 2]    = acados_params.Q[1];
  W_e[(1 + NYN) * 3]    = acados_params.Q[2];
  W_e[(1 + NYN) * 4]    = acados_params.Q[2];
  W_e[(1 + NYN) * 5]    = acados_params.Q[3];
  W_e[(1 + NYN) * 6]    = acados_params.Q[4];
  W_e[(1 + NYN) * 7]    = acados_params.Q[4];
  W_e[(1 + NYN) * 8]    = acados_params.Q[4];
  W_e[(1 + NYN) * 9]    = acados_params.Q[5];
  W_e[(1 + NYN) * 10]   = acados_params.Q[5];
  W_e[(1 + NYN) * 11]   = acados_params.Q[5];

  ocp_nlp_cost_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, N, "W", W_e);

  // --- Set Thrust Constraint
  double lg = 0;
  double ug = quadrotor_params.total_thrust_max;
  double lbu[4];
  double ubu[4];
  for (auto i = 0; i < 4; i++) {
    lbu[i] = quadrotor_params.thrust_min;
    ubu[i] = quadrotor_params.thrust_max;
  }

  for (int i = 0; i < N; i++) {
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,

                                  "lbu", lbu);
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,

                                  "ubu", ubu);

    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,

                                  "lg", &lg);
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,

                                  "ug", &ug);
  }
}
//}

/* setInitState() //{ */
void NmpcController::setInitState() {
  // --- Set Initial State in Acados
  ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, 0, "lbx",
                                x0_);
  ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, 0, "ubx",
                                x0_);
}
//}

/* setInitSolution() //{ */
void NmpcController::setInitSolution() {
  // --- Set Initialize Solution in Acados
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, acados_ocp_capsule->nlp_in, i, "x", &x0_);
    ocp_nlp_out_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, acados_ocp_capsule->nlp_in, i, "u", &u0_);
  }
  ocp_nlp_out_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, acados_ocp_capsule->nlp_in, N, "x", &x0_);
}
//}

/* setReference() //{ */
void NmpcController::setReference() {
  double yref_for_acados[NY] = {0};

  // --- Set Position Reference
  yref_for_acados[0] = yref_[states_e::x];
  yref_for_acados[1] = yref_[states_e::y];
  yref_for_acados[2] = yref_[states_e::z];

  // --- Set Attitude Error
  yref_for_acados[3] = 0;
  yref_for_acados[4] = 0;
  yref_for_acados[5] = 0;

  // --- Set Quaternion Reference
  parameters_[params_e::qw_reference] = yref_[states_e::qw];
  parameters_[params_e::qx_reference] = yref_[states_e::qx];
  parameters_[params_e::qy_reference] = yref_[states_e::qy];
  parameters_[params_e::qz_reference] = yref_[states_e::qz];

  // --- Set Speed Reference
  yref_for_acados[6] = yref_[states_e::vbx];
  yref_for_acados[7] = yref_[states_e::vby];
  yref_for_acados[8] = yref_[states_e::vbz];

  // --- Set Angular Speed Reference
  yref_for_acados[9]  = yref_[states_e::wx];
  yref_for_acados[10] = yref_[states_e::wy];
  yref_for_acados[11] = yref_[states_e::wz];

  // --- Set Initial Thrust Reference
  yref_for_acados[12] = hover_thrust_;
  yref_for_acados[13] = hover_thrust_;
  yref_for_acados[14] = hover_thrust_;
  yref_for_acados[15] = hover_thrust_;

  // --- Set Reference in Acados
  for (int i = 0; i <= N; i++) {
    ocp_nlp_cost_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, i, "yref", yref_for_acados);
    quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters_, NP);
  }
}
//}

/* setTrajectory() //{ */
void NmpcController::setTrajectory(std::vector<laser_msgs::msg::ReferenceState> trajectory) {
  double yref_for_acados[NY] = {0};

  for (auto i = 0; i <= N; i++) {
    // --- Set Position Reference
    yref_for_acados[0] = trajectory[i].use_position ? trajectory[i].pose.position.x : 0.00;
    yref_for_acados[1] = trajectory[i].use_position ? trajectory[i].pose.position.y : 0.00;
    yref_for_acados[2] = trajectory[i].use_position ? trajectory[i].pose.position.z : 0.00;

    // --- Set Quaternion Reference
    parameters_[params_e::qw_reference] = trajectory[i].use_orientation ? trajectory[i].pose.orientation.w : 1.00;
    parameters_[params_e::qx_reference] = trajectory[i].use_orientation ? trajectory[i].pose.orientation.x : 0.00;
    parameters_[params_e::qy_reference] = trajectory[i].use_orientation ? trajectory[i].pose.orientation.y : 0.00;
    parameters_[params_e::qz_reference] = trajectory[i].use_orientation ? trajectory[i].pose.orientation.z : 0.00;

    // --- Set Attitude Error
    yref_for_acados[3] = 0.00;
    yref_for_acados[4] = 0.00;
    yref_for_acados[5] = 0.00;

    // --- Set Speed Reference
    yref_for_acados[6] = trajectory[i].use_linear_velocity ? trajectory[i].twist.linear.x : 0.00;
    yref_for_acados[7] = trajectory[i].use_linear_velocity ? trajectory[i].twist.linear.y : 0.00;
    yref_for_acados[8] = trajectory[i].use_linear_velocity ? trajectory[i].twist.linear.z : 0.00;

    // --- Set Angular Speed Reference
    yref_for_acados[9]  = trajectory[i].use_angular_velocity ? trajectory[i].twist.angular.x : 0.00;
    yref_for_acados[10] = trajectory[i].use_angular_velocity ? trajectory[i].twist.angular.y : 0.00;
    yref_for_acados[11] = trajectory[i].use_angular_velocity ? trajectory[i].twist.angular.z : 0.00;

    // --- Set Initial Thrust Reference
    yref_for_acados[12] = hover_thrust_;
    yref_for_acados[13] = hover_thrust_;
    yref_for_acados[14] = hover_thrust_;
    yref_for_acados[15] = hover_thrust_;

    // --- Set Reference in Acados
    ocp_nlp_cost_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, i, "yref", yref_for_acados);
    quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters_, NP);
  }
}
//}

/* ocpSolver() //{ */
bool NmpcController::ocpSolver() {
  // --- Sets the first iteration phase that will start solving the problem
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_opts, "rti_phase", &rti_phase);
  int status = quadrotor_ode_acados_solve(acados_ocp_capsule);

  if (status != ACADOS_SUCCESS) {
    printf("quadrotor_ode_acados_solve() failed with status %d.\n", status);
    return false;
  }
  return true;
}
//}

/* printStatistics() //{ */
void NmpcController::printStatistics() {
  int    sqp_iter, stat_m, stat_n;
  double elapsed_time;
  double nlp_res;
  double cost_value;
  double stat[1200];

  ocp_nlp_eval_cost(acados_ocp_capsule->nlp_solver, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out);

  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "sqp_iter", &sqp_iter);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "stat_n", &stat_n);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "stat_m", &stat_m);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "statistics", stat);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "time_tot", &elapsed_time);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "nlp_res", &nlp_res);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "cost_value", &cost_value);

  int qp_iter = (int)stat[2 + 1 * 3];

  std::cout << "qp iterations: " << qp_iter << ", time elapsed: " << elapsed_time * 1000 << " ms."
            << ", cost value: " << cost_value << std::endl;
}
//}

///* getFirstControlInput() //{ */
void NmpcController::getFirstControlInput() {
  ocp_nlp_out_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, 0, "u", &u0_);
}
//}

///* getFirstComputedStates() //{ */
void NmpcController::getFirstComputedStates() {
  ocp_nlp_out_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, 1, "x", &x1_);
}
//}

///* printAllComputedStates() //{ */
void NmpcController::printAllComputedStates() {
  double x[NX];
  for (int i = 0; i <= N; i++) {
    ocp_nlp_out_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, i, "x", &x);

    std::cout << "N = " << i << std::endl;
    for (auto j = 0; j < NX; j++) {
      std::cout << x[j] << std::endl;
    }
    std::cout << std::endl << std::endl;
  }
}
//}

///* printAllControlInputs() //{ */
void NmpcController::printAllControlInputs() {
  double u[NU];

  for (int i = 0; i < N; i++) {
    ocp_nlp_out_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_out, i, "u", &u);

    std::cout << "N = " << i << std::endl;
    for (auto j = 0; j < NU; j++) {
      std::cout << u[j] << std::endl;
    }
    std::cout << std::endl << std::endl;
  }
}
//}

/* thrustToThrotle() //{ */
double NmpcController::thrustToThrotle() {
  // --- The normalization is maded by quadratic curve that represent thrust force x throtle normalized
  return (motor_curve_a_ * sqrt(((u0_[control_input_e::w1] + u0_[control_input_e::w2] + u0_[control_input_e::w3] + u0_[control_input_e::w4]) / 4))) +
         motor_curve_b_;
}
//}

///* getCorrection() //{ */
laser_msgs::msg::AttitudeRatesAndThrust NmpcController::getCorrection(laser_msgs::msg::ReferenceState reference, const nav_msgs::msg::Odometry msg) {
  // --- Update reference
  yref_[states_e::x]   = reference.use_position ? reference.pose.position.x : 0.00;
  yref_[states_e::y]   = reference.use_position ? reference.pose.position.y : 0.00;
  yref_[states_e::z]   = reference.use_position ? reference.pose.position.z : 0.00;
  yref_[states_e::qw]  = reference.use_orientation ? reference.pose.orientation.w : 1.00;
  yref_[states_e::qx]  = reference.use_orientation ? reference.pose.orientation.x : 0.00;
  yref_[states_e::qy]  = reference.use_orientation ? reference.pose.orientation.y : 0.00;
  yref_[states_e::qz]  = reference.use_orientation ? reference.pose.orientation.z : 0.00;
  yref_[states_e::vbx] = reference.use_linear_velocity ? reference.twist.linear.x : 0.00;
  yref_[states_e::vby] = reference.use_linear_velocity ? reference.twist.linear.y : 0.00;
  yref_[states_e::vbz] = reference.use_linear_velocity ? reference.twist.linear.z : 0.00;
  yref_[states_e::wx]  = reference.use_angular_velocity ? reference.twist.angular.x : 0.00;
  yref_[states_e::wy]  = reference.use_angular_velocity ? reference.twist.angular.y : 0.00;
  yref_[states_e::wz]  = reference.use_angular_velocity ? reference.twist.angular.z : 0.00;

  // --- Init Thrust is hover thrust assumption
  u0_[control_input_e::w1] = hover_thrust_;
  u0_[control_input_e::w2] = hover_thrust_;
  u0_[control_input_e::w3] = hover_thrust_;
  u0_[control_input_e::w4] = hover_thrust_;

  // --- Read Estimate
  x0_[states_e::x]   = msg.pose.pose.position.x;
  x0_[states_e::y]   = msg.pose.pose.position.y;
  x0_[states_e::z]   = msg.pose.pose.position.z;
  x0_[states_e::qw]  = msg.pose.pose.orientation.w;
  x0_[states_e::qx]  = msg.pose.pose.orientation.x;
  x0_[states_e::qy]  = msg.pose.pose.orientation.y;
  x0_[states_e::qz]  = msg.pose.pose.orientation.z;
  x0_[states_e::vbx] = msg.twist.twist.linear.x;
  x0_[states_e::vby] = msg.twist.twist.linear.y;
  x0_[states_e::vbz] = msg.twist.twist.linear.z;
  x0_[states_e::wx]  = msg.twist.twist.angular.x;
  x0_[states_e::wy]  = msg.twist.twist.angular.y;
  x0_[states_e::wz]  = msg.twist.twist.angular.z;

  // --- Initialize solution
  setInitState();
  setInitSolution();
  setReference();

  // --- Solver OCP
  if (ocpSolver()) {
    printStatistics();

    // --- Take the optimum control input and computed state
    getFirstControlInput();
    getFirstComputedStates();
  }

  // --- Fill the input control message to pixhawk
  laser_msgs::msg::AttitudeRatesAndThrust control_input;
  control_input.roll_rate               = x1_[states_e::wx];
  control_input.pitch_rate              = x1_[states_e::wy];
  control_input.yaw_rate                = x1_[states_e::wz];
  control_input.total_thrust_normalized = thrustToThrotle();

  std::cout << "Control Input" << std::endl;
  std::cout << "w1: " << u0_[control_input_e::w1] << ", w2: " << u0_[control_input_e::w2] << ", w3: " << u0_[control_input_e::w3]
            << ", w4: " << u0_[control_input_e::w4] << std::endl;

  return control_input;
}
//}

///* getCorrection() //{ */
laser_msgs::msg::AttitudeRatesAndThrust NmpcController::getCorrection(std::vector<laser_msgs::msg::ReferenceState> trajectory,
                                                                      const nav_msgs::msg::Odometry                msg) {
  // --- Init Thrust is hover thrust assumption
  u0_[control_input_e::w1] = hover_thrust_;
  u0_[control_input_e::w2] = hover_thrust_;
  u0_[control_input_e::w3] = hover_thrust_;
  u0_[control_input_e::w4] = hover_thrust_;

  // --- Read Estimate
  x0_[states_e::x]   = msg.pose.pose.position.x;
  x0_[states_e::y]   = msg.pose.pose.position.y;
  x0_[states_e::z]   = msg.pose.pose.position.z;
  x0_[states_e::qw]  = msg.pose.pose.orientation.w;
  x0_[states_e::qx]  = msg.pose.pose.orientation.x;
  x0_[states_e::qy]  = msg.pose.pose.orientation.y;
  x0_[states_e::qz]  = msg.pose.pose.orientation.z;
  x0_[states_e::vbx] = msg.twist.twist.linear.x;
  x0_[states_e::vby] = msg.twist.twist.linear.y;
  x0_[states_e::vbz] = msg.twist.twist.linear.z;
  x0_[states_e::wx]  = msg.twist.twist.angular.x;
  x0_[states_e::wy]  = msg.twist.twist.angular.y;
  x0_[states_e::wz]  = msg.twist.twist.angular.z;

  // --- Initialize solution
  setInitState();
  setInitSolution();
  setTrajectory(trajectory);

  // --- Solver OCP
  if (ocpSolver()) {
    printStatistics();

    // --- Take the optimum control input and computed state
    getFirstControlInput();
    getFirstComputedStates();
  }

  // --- Fill the input control message to pixhawk
  laser_msgs::msg::AttitudeRatesAndThrust control_input;
  control_input.roll_rate               = x1_[states_e::wx];
  control_input.pitch_rate              = x1_[states_e::wy];
  control_input.yaw_rate                = x1_[states_e::wz];
  control_input.total_thrust_normalized = thrustToThrotle();

  std::cout << "Control Input" << std::endl;
  std::cout << "w1: " << u0_[control_input_e::w1] << ", w2: " << u0_[control_input_e::w2] << ", w3: " << u0_[control_input_e::w3]
            << ", w4: " << u0_[control_input_e::w4] << std::endl;

  return control_input;
}
//}
}  // namespace laser_uav_controllers
