#include <laser_uav_controllers/nmpc_controller.hpp>

namespace laser_uav_controllers
{
/* NmpcController() {default} //{ */
NmpcController::NmpcController() {
}
//}

/* NmpcController() //{ */
NmpcController::NmpcController(multirotor_t multirotor_params, acados_t acados_params) {
  if (acados_params.nmpc_mode == "individual_thrust") {
    angular_rates_and_thrust_mode_ = false;
  } else {
    angular_rates_and_thrust_mode_ = true;
  }

  std::cout << "creating acados ocp solver" << std::endl;
  acados_ocp_capsule = multirotor_ode_acados_create_capsule();
  N                  = acados_params.N;
  dt_rvc_            = acados_params.dt;

  double* dt = new double[N];
  std::fill_n(dt, N, (N * acados_params.dt) / N);
  int status = multirotor_ode_acados_create_with_discretization(acados_ocp_capsule, N, dt);

  if (status) {
    printf("creation of acados ocp solver returned status %d. Exiting.\n", status);
    exit(1);
  }

  n_motors_ = multirotor_params.n_motors;

  parameters_[params_e::mass] = multirotor_params.mass;
  for (auto i = 0; i < multirotor_params.G1.rows(); i++) {
    auto j = i * 8;
    for (auto k = 0; k < multirotor_params.G1.cols(); k++) {
      parameters_[params_e::G1 + j + k] = multirotor_params.G1(i, k);
    }
  }
  Eigen::Vector3d aux = multirotor_params.inertia_matrix.diagonal();
  std::copy(aux.data(), aux.data() + aux.size(), &parameters_[params_e::inertia]);
  std::copy(multirotor_params.drag.data(), multirotor_params.drag.data() + multirotor_params.drag.size(), &parameters_[params_e::drag]);

  /* // --- Set Nominal Quaternion in Initialize */
  parameters_[params_e::qw_reference] = 1;
  parameters_[params_e::qx_reference] = 0;
  parameters_[params_e::qy_reference] = 0;
  parameters_[params_e::qz_reference] = 0;

  motor_curve_a_ = multirotor_params.motor_curve_a;
  motor_curve_b_ = multirotor_params.motor_curve_b;

  // --- Calculate Hover Thrust in Newtons
  hover_thrust_ = (parameters_[params_e::mass] * GRAVITY) / n_motors_;

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

  for (auto i = 0; i < 8; i++) {
    if (i < n_motors_) {
      W[(1 + NY) * (i + 12)] = acados_params.R;
    } else {
      W[(1 + NY) * (i + 12)] = 0.0;
    }
  }

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

  /* // --- Set Omega Constrint */
  double omega_max[3];
  omega_max[0] = multirotor_params.omega_max(0);
  omega_max[1] = multirotor_params.omega_max(1);
  omega_max[2] = multirotor_params.omega_max(2);

  double omega_min[3];
  omega_min[0] = -multirotor_params.omega_max(0);
  omega_min[1] = -multirotor_params.omega_max(1);
  omega_min[2] = -multirotor_params.omega_max(2);

  for (int i = 0; i < N; i++) {
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,
                                  "ubx", omega_max);
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,
                                  "lbx", omega_min);
  }

  /* // --- Set Thrust Constraint */
  double lg[1] = {0};
  double ug[1] = {multirotor_params.total_thrust_max};
  double lbu[8];
  double ubu[8];
  for (auto i = 0; i < 8; i++) {
    if (i < n_motors_) {
      lbu[i] = multirotor_params.thrust_min;
      ubu[i] = multirotor_params.thrust_max;
    } else {
      lbu[i] = 0;
      ubu[i] = 0;
    }
  }

  for (int i = 0; i < N; i++) {
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,
                                  "lbu", lbu);
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,
                                  "ubu", ubu);

    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,
                                  "lg", lg);
    ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, i,
                                  "ug", ug);
  }
}
//}

/* setInitState() //{ */
void NmpcController::setInitState() {
  // --- Set Initial State in Acados
  ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, 0, "ubx",
                                x0_);
  ocp_nlp_constraints_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, acados_ocp_capsule->nlp_out, 0, "lbx",
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

/* setTrajectory() //{ */
void NmpcController::setTrajectory(std::vector<laser_msgs::msg::ReferenceState> trajectory) {
  double yref_for_acados[NY] = {0};

  for (auto k = 0; k <= N; k++) {
    auto j = k;
    if ((int)trajectory.size() != N + 1) {
      j = 0;
    }

    // --- Set Position Reference
    yref_for_acados[0] = trajectory[j].use_position ? trajectory[j].pose.position.x : 0.00;
    yref_for_acados[1] = trajectory[j].use_position ? trajectory[j].pose.position.y : 0.00;
    yref_for_acados[2] = trajectory[j].use_position ? trajectory[j].pose.position.z : 0.00;

    // --- Set Quaternion Reference
    parameters_[params_e::qw_reference] = trajectory[j].use_orientation ? trajectory[j].pose.orientation.w : 1.00;
    parameters_[params_e::qx_reference] = trajectory[j].use_orientation ? trajectory[j].pose.orientation.x : 0.00;
    parameters_[params_e::qy_reference] = trajectory[j].use_orientation ? trajectory[j].pose.orientation.y : 0.00;
    parameters_[params_e::qz_reference] = trajectory[j].use_orientation ? trajectory[j].pose.orientation.z : 0.00;

    // --- Set Attitude Error
    yref_for_acados[3] = 0.00;
    yref_for_acados[4] = 0.00;
    yref_for_acados[5] = 0.00;

    // --- Set Speed Reference
    yref_for_acados[6] = trajectory[j].use_linear_velocity ? trajectory[j].twist.linear.x : 0.00;
    yref_for_acados[7] = trajectory[j].use_linear_velocity ? trajectory[j].twist.linear.y : 0.00;
    yref_for_acados[8] = trajectory[j].use_linear_velocity ? trajectory[j].twist.linear.z : 0.00;

    // --- Set Angular Speed Reference
    yref_for_acados[9]  = trajectory[j].use_angular_velocity ? trajectory[j].twist.angular.x : 0.00;
    yref_for_acados[10] = trajectory[j].use_angular_velocity ? trajectory[j].twist.angular.y : 0.00;
    yref_for_acados[11] = trajectory[j].use_angular_velocity ? trajectory[j].twist.angular.z : 0.00;

    // --- Set Initial Thrust Reference (Variável m para motores)
    for (auto m = 0; m < 8; m++) {
      if (m < n_motors_) {
        yref_for_acados[m + 12] = trajectory[j].use_individual_thrust ? trajectory[j].individual_thrust.data[m] : hover_thrust_;
      } else {
        yref_for_acados[m + 12] = 0.0;
      }
    }

    double t_k = k * dt_rvc_;

    for (int idx = 0; idx < 15; idx++) {
      if (rvc_tv_m_[idx / 3] > t_k) {
        parameters_[params_e::rvc_Am_start + idx] = rvc_Am_[idx];
      } else {
        parameters_[params_e::rvc_Am_start + idx] = 0.0;
      }
    }

    for (int idx = 0; idx < 5; idx++) {
      if (rvc_tv_m_[idx] > t_k) {
        parameters_[params_e::rvc_bm_start + idx] = rvc_bm_[idx];
      } else {
        parameters_[params_e::rvc_bm_start + idx] = -100.0;
      }
    }

    ocp_nlp_cost_model_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_dims, acados_ocp_capsule->nlp_in, k, "yref", yref_for_acados);
    multirotor_ode_acados_update_params(acados_ocp_capsule, k, parameters_, NP);
  }
}
//}

/* setRVCConstraints() //{ */
void NmpcController::setRVCConstraints(const std::vector<double>& Am, const std::vector<double>& bm, const std::vector<double>& tv_m) {
  rvc_Am_   = Am;
  rvc_bm_   = bm;
  rvc_tv_m_ = tv_m;

  for (int i = 0; i < 15; i++) {
    parameters_[rvc_Am_start + i] = rvc_Am_[i];
  }

  for (int i = 0; i < 5; i++) {
    parameters_[rvc_bm_start + i] = rvc_bm_[i];
  }
}
//}

/* ocpSolver() //{ */
bool NmpcController::ocpSolver() {
  // --- Sets the first iteration phase that will start solving the problem
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_opts, "rti_phase", &rti_phase);
  int status = multirotor_ode_acados_solve(acados_ocp_capsule);

  if (status != ACADOS_SUCCESS) {
    printf("multirotor_ode_acados_solve() failed with status %d.\n", status);
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

/* getTotalElapsedTime() //{ */
double NmpcController::getOcpElapsedTime() {
  double elapsed_time;
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "time_tot", &elapsed_time);

  return elapsed_time * 1000;
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

///* getLastIndividualThrust() //{ */
std::vector<double> NmpcController::getLastIndividualThrust() {
  std::vector<double> aux;
  aux.push_back(u0_[control_input_e::w1]);
  aux.push_back(u0_[control_input_e::w2]);
  aux.push_back(u0_[control_input_e::w3]);
  aux.push_back(u0_[control_input_e::w4]);

  return aux;
}
//}

///* setMass() //{ */
void NmpcController::setMass(double mass) {
  parameters_[params_e::mass] = mass;
  hover_thrust_               = (parameters_[params_e::mass] * GRAVITY) / n_motors_;
}
//}

///* getCorrection() //{ */
std::pair<Eigen::Vector3d, Eigen::VectorXd> NmpcController::getCorrection(laser_msgs::msg::ReferenceState reference, const nav_msgs::msg::Odometry msg) {
  // --- Init Thrust is hover thrust assumption
  for (auto i = 0; i < 8; i++) {
    if (i < n_motors_) {
      u0_[i] = hover_thrust_;
    } else {
      u0_[i] = 0.0;
    }
  }

  // --- Read Estimate
  x0_[states_e::x]  = msg.pose.pose.position.x;
  x0_[states_e::y]  = msg.pose.pose.position.y;
  x0_[states_e::z]  = msg.pose.pose.position.z;
  x0_[states_e::qw] = msg.pose.pose.orientation.w;
  x0_[states_e::qx] = msg.pose.pose.orientation.x;
  x0_[states_e::qy] = msg.pose.pose.orientation.y;
  x0_[states_e::qz] = msg.pose.pose.orientation.z;

  Eigen::Vector3d    twist_world_frame(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  Eigen::Quaterniond q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  twist_world_frame  = q.normalized().toRotationMatrix() * twist_world_frame;
  x0_[states_e::vbx] = twist_world_frame(0);
  x0_[states_e::vby] = twist_world_frame(1);
  x0_[states_e::vbz] = twist_world_frame(2);

  x0_[states_e::wx] = msg.twist.twist.angular.x;
  x0_[states_e::wy] = msg.twist.twist.angular.y;
  x0_[states_e::wz] = msg.twist.twist.angular.z;

  // --- Initialize solution
  setInitState();
  setInitSolution();
  std::vector<laser_msgs::msg::ReferenceState> trajectory;
  trajectory.push_back(reference);
  setTrajectory(trajectory);

  // --- Solver OCP
  if (ocpSolver()) {
    // --- Take the optimum control input and computed state
    getFirstControlInput();
    getFirstComputedStates();
  }

  Eigen::Vector3d desired_angular_speed;
  Eigen::VectorXd control_input;

  desired_angular_speed(0) = x1_[states_e::wx];
  desired_angular_speed(1) = x1_[states_e::wy];
  desired_angular_speed(2) = x1_[states_e::wz];

  control_input = Eigen::VectorXd::Zero(n_motors_);
  for (auto i = 0; i < n_motors_; i++) {
    control_input(i) = u0_[i];
  }

  return {desired_angular_speed, control_input};
}
//}

///* getCorrection() //{ */
std::pair<Eigen::Vector3d, Eigen::VectorXd> NmpcController::getCorrection(std::vector<laser_msgs::msg::ReferenceState> trajectory,
                                                                          const nav_msgs::msg::Odometry                msg) {
  // --- Init Thrust is hover thrust assumption
  for (auto i = 0; i < 8; i++) {
    if (i < n_motors_) {
      u0_[i] = hover_thrust_;
    } else {
      u0_[i] = 0.0;
    }
  }

  // --- Read Estimate
  x0_[states_e::x]  = msg.pose.pose.position.x;
  x0_[states_e::y]  = msg.pose.pose.position.y;
  x0_[states_e::z]  = msg.pose.pose.position.z;
  x0_[states_e::qw] = msg.pose.pose.orientation.w;
  x0_[states_e::qx] = msg.pose.pose.orientation.x;
  x0_[states_e::qy] = msg.pose.pose.orientation.y;
  x0_[states_e::qz] = msg.pose.pose.orientation.z;

  Eigen::Vector3d    twist_world_frame(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  Eigen::Quaterniond q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  twist_world_frame  = q.normalized().toRotationMatrix() * twist_world_frame;
  x0_[states_e::vbx] = twist_world_frame(0);
  x0_[states_e::vby] = twist_world_frame(1);
  x0_[states_e::vbz] = twist_world_frame(2);

  x0_[states_e::wx] = msg.twist.twist.angular.x;
  x0_[states_e::wy] = msg.twist.twist.angular.y;
  x0_[states_e::wz] = msg.twist.twist.angular.z;

  // --- Initialize solution
  setInitState();
  setInitSolution();
  setTrajectory(trajectory);

  // --- Solver OCP
  if (ocpSolver()) {
    // --- Take the optimum control input and computed state
    getFirstControlInput();
    getFirstComputedStates();
  }

  Eigen::Vector3d desired_angular_speed;
  Eigen::VectorXd control_input;

  desired_angular_speed(0) = x1_[states_e::wx];
  desired_angular_speed(1) = x1_[states_e::wy];
  desired_angular_speed(2) = x1_[states_e::wz];

  control_input = Eigen::VectorXd::Zero(n_motors_);
  for (auto i = 0; i < n_motors_; i++) {
    control_input(i) = u0_[i];
  }

  return {desired_angular_speed, control_input};
}
//}
}  // namespace laser_uav_controllers
