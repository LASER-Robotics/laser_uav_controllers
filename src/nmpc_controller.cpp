#include <laser_uav_controllers/nmpc_controller.hpp>
#include <stdio.h>

namespace laser_uav_controllers
{
/* NmpcController() //{ */
NmpcController::NmpcController() {
  int     _N             = 20;
  double *new_time_steps = NULL;
  double *dt;

  std::cout << "creating acados_drone" << std::endl;
  acados_ocp_capsule = quadrotor_ode_acados_create_capsule();
  // there is an opportunity to change the number of shooting intervals in C
  // without new code generation
  int status = quadrotor_ode_acados_create_with_discretization(acados_ocp_capsule, _N, new_time_steps);

  if (status) {
    printf("quadrotor_ode_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  if (new_time_steps == NULL) {
    dt = new double[_N];
    std::fill_n(dt, _N, 2.0 / 30.0);
  } else {
    dt = new double[_N];
    for (int i = 0; i < _N; i++) {
      dt[i] = new_time_steps[i];
    }
  }

  // initialize parameters to nominal value
  parameters_[0]  = 2.76;     // mass
  parameters_[1]  = -0.185;   // mot0_pos[0]
  parameters_[2]  = -0.18;    // mot0_pos[1]
  parameters_[3]  = 0.185;    // mot1_pos[0]
  parameters_[4]  = -0.18;    // mot1_pos[1]
  parameters_[5]  = -0.185;   // mot2_pos[0]
  parameters_[6]  = 0.18;     // mot2_pos[1]
  parameters_[7]  = 0.185;    // mot3_pos[0]
  parameters_[8]  = 0.18;     // mot3_pos[1]
  parameters_[9]  = 0.4953;   // inertia x
  parameters_[10] = 0.4953;   // inertia y
  parameters_[11] = 0.3413;   // inertia z
  parameters_[12] = 7.97e-8;  // ctau
  parameters_[13] = 0;        // drag
  parameters_[14] = 0;        // drag
  parameters_[15] = 0;        // drag
  parameters_[16] = 1.0;      // q_ref_w
  parameters_[17] = 0.0;      // q_ref_x
  parameters_[18] = 0.0;      // q_ref_y
  parameters_[19] = 0.0;      // q_ref_z

  nlp_config = quadrotor_ode_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims   = quadrotor_ode_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in     = quadrotor_ode_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out    = quadrotor_ode_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = quadrotor_ode_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts   = quadrotor_ode_acados_get_nlp_opts(acados_ocp_capsule);


  /* int idxbx0[NBX0]; */
  /* for (int i = 0; i < NBX0; i++) { */
  /*   idxbx0[i] = i; */
  /* } */
  /* ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0); */

  hover_thrust_ = (parameters_[mass] * GRAVITY) / 4;

  /* std::cout << "creating acados_drone from DroneParams and AcadosParams structs" << std::endl; */
  /* acados_ocp_capsule = quadrotor_ode_acados_create_capsule(); */

  /* // set default */
  /* N                      = acParams.ac_num_iter; */
  /* const Scalar acados_tf = acParams.ac_horizon; */
  /* dt                     = new double[N]; */
  /* std::fill_n(dt, N, acados_tf / N); */

  /* double *new_time_steps = NULL; */
  /* ref_thrust             = 3; */

  /* double xtraj[13 * (N + 1)]; */
  /* double utraj[13 * N]; */

  /* double time_sum = 0; */
  /* double time_max = 0; */
  /* new_time_steps  = new double[N]; */
  /* for (int i = 0; i < N; i++) { */
  /*   new_time_steps[i] = acParams.ac_time_steps[i]; */
  /*   dt[i]             = acParams.ac_time_steps[i]; */
  /* } */

  /* // there is an opportunity to change the number of shooting intervals in C */
  /* // without new code generation */
  /* int status = quadrotor_ode_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps); */

  /* if (status) { */
  /*   printf("quadrotor_ode_acados_create() returned status %d. Exiting.\n", status); */
  /*   exit(1); */
  /* } */

  /* if (new_time_steps != NULL) { */
  /*   delete (new_time_steps); */
  /* } */

  /* nlp_config = quadrotor_ode_acados_get_nlp_config(acados_ocp_capsule); */
  /* nlp_dims   = quadrotor_ode_acados_get_nlp_dims(acados_ocp_capsule); */
  /* nlp_in     = quadrotor_ode_acados_get_nlp_in(acados_ocp_capsule); */
  /* nlp_out    = quadrotor_ode_acados_get_nlp_out(acados_ocp_capsule); */
  /* nlp_solver = quadrotor_ode_acados_get_nlp_solver(acados_ocp_capsule); */
  /* nlp_opts   = quadrotor_ode_acados_get_nlp_opts(acados_ocp_capsule); */

  /* // parse drone configuration */
  /* parameters[0] = uavParams.mass; */
  /* ref_thrust    = parameters[0] * 9.806 / 4; */

  /* parameters[1] = uavParams.mot0_pos[0];  // mot0_pos[0] */
  /* parameters[2] = uavParams.mot0_pos[1];  // mot0_pos[1] */
  /* parameters[3] = uavParams.mot1_pos[0];  // mot1_pos[0] */
  /* parameters[4] = uavParams.mot1_pos[1];  // mot1_pos[1] */
  /* parameters[5] = uavParams.mot2_pos[0];  // mot2_pos[0] */
  /* parameters[6] = uavParams.mot2_pos[1];  // mot2_pos[1] */
  /* parameters[7] = uavParams.mot3_pos[0];  // mot3_pos[0] */
  /* parameters[8] = uavParams.mot3_pos[1];  // mot3_pos[1] */

  /* parameters[9]  = uavParams.inertia_diag[0]; */
  /* parameters[10] = uavParams.inertia_diag[1]; */
  /* parameters[11] = uavParams.inertia_diag[2]; */

  /* parameters[12] = uavParams.ctau; */
  /* parameters[13] = uavParams.drag[0]; */
  /* parameters[14] = uavParams.drag[1]; */
  /* parameters[15] = uavParams.drag[2]; */

  /* // initialize parameters to nominal value */
  /* parameters[16] = 1.0;  // q_ref_w */
  /* parameters[17] = 0.0;  // q_ref_x */
  /* parameters[18] = 0.0;  // q_ref_y */
  /* parameters[19] = 0.0;  // q_ref_z */

  /* for (int i = 0; i <= N; i++) { */
  /*   quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters, NP); */
  /* } */

  /* // parse optimizer matrices */
  /* double *W     = new double[NY * NY](); */
  /* W[0 + (NY)*0] = acParams.Q_pos_xy; */
  /* W[1 + (NY)*1] = acParams.Q_pos_xy; */
  /* W[2 + (NY)*2] = acParams.Q_pos_z; */
  /* W[3 + (NY)*3] = acParams.Q_rot_xy; */
  /* ; */
  /* W[4 + (NY)*4] = acParams.Q_rot_xy; */
  /* ; */
  /* W[5 + (NY)*5] = acParams.Q_rot_z; */
  /* ; */
  /* W[6 + (NY)*6]   = acParams.Q_vel; */
  /* W[7 + (NY)*7]   = acParams.Q_vel; */
  /* W[8 + (NY)*8]   = acParams.Q_vel; */
  /* W[9 + (NY)*9]   = acParams.Q_ome; */
  /* W[10 + (NY)*10] = acParams.Q_ome; */
  /* W[11 + (NY)*11] = acParams.Q_ome; */
  /* W[12 + (NY)*12] = acParams.R_thr; */
  /* W[13 + (NY)*13] = acParams.R_thr; */
  /* W[14 + (NY)*14] = acParams.R_thr; */
  /* W[15 + (NY)*15] = acParams.R_thr; */

  /* printf( */
  /*     "Costs: Q_pos_xy:%f, Q_pos_z:%f, Q_rot_xy:%f, Q_rot_z:%f, " */
  /*     "Q_vel:%f,Q_rot:%f,R_thr:%f\n", */
  /*     acParams.Q_pos_xy, acParams.Q_pos_z, acParams.Q_rot_xy, acParams.Q_rot_z, acParams.Q_vel, acParams.Q_ome, acParams.R_thr); */
  /* for (int i = 0; i < N; i++) { */
  /*   ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W); */
  /* } */
  /* delete (W); */

  /* double *W_e = new double[NYN * NYN](); */
  /* // change only the non-zero elements: */
  /* W_e[0 + (NYN)*0] = acParams.Q_pos_xy; */
  /* W_e[1 + (NYN)*1] = acParams.Q_pos_xy; */
  /* W_e[2 + (NYN)*2] = acParams.Q_pos_z; */
  /* W_e[3 + (NYN)*3] = acParams.Q_rot_xy; */
  /* ; */
  /* W_e[4 + (NYN)*4] = acParams.Q_rot_xy; */
  /* ; */
  /* W_e[5 + (NYN)*5] = acParams.Q_rot_z; */
  /* ; */
  /* W_e[6 + (NYN)*6]   = acParams.Q_vel; */
  /* W_e[7 + (NYN)*7]   = acParams.Q_vel; */
  /* W_e[8 + (NYN)*8]   = acParams.Q_vel; */
  /* W_e[9 + (NYN)*9]   = acParams.Q_ome; */
  /* W_e[10 + (NYN)*10] = acParams.Q_ome; */
  /* W_e[11 + (NYN)*11] = acParams.Q_ome; */

  /* ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e); */

  /* delete (W_e); */

  /* // parse constrains: */
  /* double thrust_min = uavParams.thrust_min; */
  /* double thrust_max = uavParams.thrust_max; */
  /* int   *idxbu      = new int[NBU]; */

  /* idxbu[0]     = 0; */
  /* idxbu[1]     = 1; */
  /* idxbu[2]     = 2; */
  /* idxbu[3]     = 3; */
  /* double *lubu = new double[2 * NBU](); */
  /* double *lbu  = lubu; */
  /* double *ubu  = lubu + NBU; */

  /* lbu[0] = thrust_min; */
  /* lbu[1] = thrust_min; */
  /* lbu[2] = thrust_min; */
  /* lbu[3] = thrust_min; */
  /* ubu[0] = thrust_max; */
  /* ubu[1] = thrust_max; */
  /* ubu[2] = thrust_max; */
  /* ubu[3] = thrust_max; */

  /* ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbu", idxbu); */
  /* ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbu", lbu); */
  /* ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", ubu); */
  /* /1* ubu[0] = thrust_max * 0.9; *1/ */
  /* /1* ubu[1] = thrust_max * 0.9; *1/ */
  /* /1* ubu[2] = thrust_max * 0.9; *1/ */
  /* /1* ubu[3] = thrust_max * 0.9; *1/ */
  /* /1* TODO: investigate why there was 90 percent of max thrust ... *1/ */

  /* for (int i = 1; i < N; i++) { */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu); */
  /* } */
  /* delete (idxbu); */
  /* delete (lubu); */

  /* double *C   = new double[NG * NX](); */
  /* double *D   = new double[NG * NU](); */
  /* double *lug = new double[2 * NG](); */
  /* double *lg  = lug; */
  /* double *ug  = lug + NG; */

  /* D[0 + NG * 0] = 1; */
  /* D[0 + NG * 1] = 1; */
  /* D[0 + NG * 2] = 1; */
  /* D[0 + NG * 3] = 1; */

  /* ug[0] = uavParams.thrust_total_max; */
  /* lg[0] = uavParams.thrust_total_min; */

  /* for (int i = 0; i < N; i++) { */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "D", D); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "C", C); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lg", lg); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ug", ug); */
  /* } */
  /* free(D); */
  /* free(C); */
  /* free(lug); */

  /* int *idxbx = new int[NBX]; */

  /* idxbx[0]     = 10; */
  /* idxbx[1]     = 11; */
  /* idxbx[2]     = 12; */
  /* double *lubx = new double[2 * NBX]; */
  /* double *lbx  = lubx; */
  /* double *ubx  = lubx + NBX; */

  /* lbx[0] = -uavParams.max_rotation_speed[0]; */
  /* ubx[0] = uavParams.max_rotation_speed[0]; */
  /* lbx[1] = -uavParams.max_rotation_speed[1]; */
  /* ubx[1] = uavParams.max_rotation_speed[1]; */
  /* lbx[2] = -uavParams.max_rotation_speed[2]; */
  /* ubx[2] = uavParams.max_rotation_speed[2]; */

  /* for (int i = 1; i < N; i++) { */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx); */
  /*   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx); */
  /* } */
  /* delete (idxbx); */
  /* delete (lubx); */
}
//}

/* setInitState() //{ */
void NmpcController::setInitState() {
  int idxbx0[NBX0];
  for (int i = 0; i < NBX0; i++) {
    idxbx0[i] = i;
  }
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0_);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0_);

  double max_thrust[4] = {15.7, 15.7, 15.7, 15.7};
  double min_thrust[4] = {0.0};
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbu", &min_thrust);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubu", &max_thrust);

  double total_max_thrust[4] = {max_thrust[0] * 4, max_thrust[0] * 4, max_thrust[0] * 4, max_thrust[0] * 4};
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lg", &min_thrust);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ug", &total_max_thrust);
}
//}

/* setInitSolution() //{ */
void NmpcController::setInitSolution() {
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", &x0_);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", &u0_);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", &x0_);
}
//}

/* setReference() //{ */
void NmpcController::setReference() {
  double yref_for_acados[NY] = {0};

  // Set Position Reference
  for (int i = 0; i < 3; i++) {
    yref_for_acados[i] = yref_[i];
  }

  // Set Attitude Error Reference
  for (int i = 3; i < 6; i++) {
    yref_for_acados[i] = 0;
  }

  // Set Velocity and Angular Velocity Reference
  for (int i = 6; i < NYN; i++) {
    yref_for_acados[i] = yref_[i + 1];
  }

  for (int i = 0; i < NU; i++) {
    yref_for_acados[NYN + i] = hover_thrust_;
  }

  for (int i = 0; i <= N; i++) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_for_acados);
  }

  parameters_[params_e::qw_reference] = yref_[states_e::qw];
  parameters_[params_e::qx_reference] = yref_[states_e::qx];
  parameters_[params_e::qy_reference] = yref_[states_e::qy];
  parameters_[params_e::qz_reference] = yref_[states_e::qz];

  for (int i = 0; i <= N; i++) {
    quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters_, NP);
  }
}
//}

/* ocpSolver() //{ */
bool NmpcController::ocpSolver() {
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
  int status = quadrotor_ode_acados_solve(acados_ocp_capsule);

  if (status != ACADOS_SUCCESS) {
    printf("quadrotor_ode_acados_solve() failed with status %d.\n", status);
    return false;
  }
  return true;
}
//}

/* printStats() //{ */
void NmpcController::printStats() {
  int sqp_iter, stat_m, stat_n, tmp_int;
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "sqp_iter", &sqp_iter);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "stat_n", &stat_n);
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "stat_m", &stat_m);

  double stat[1200];
  ocp_nlp_get(acados_ocp_capsule->nlp_solver, "statistics", stat);

  int qp_iter = (int)stat[2 + 1 * 3];
  /* double elapsed_time = get_elapsed_time(); */
  /* std::cout << "qp iterations: " << qp_iter << ", time elapsed: " << elapsed_time * 1000 << " ms." << std::endl; */
  std::cout << "qp iterations: " << qp_iter << std::endl;
}
//}

///* getFirstControlInput() //{ */
void NmpcController::getFirstControlInput() {
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u0_);
}
//}

///* get_all_computed_states() //{ */
void NmpcController::getAllComputedStates() {
  double x[NX];

  for (int i = 0; i <= N; i++) {
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &x);

    std::cout << "N = " << i << std::endl;
    for (auto j = 0; j < NX; j++) {
      std::cout << x[j] << std::endl;
    }
    std::cout << std::endl << std::endl;
    /* output.push_back(a); */
  }
  /* return output; */
}
//}

///* get_all_computed_states() //{ */
void NmpcController::getFirstComputedStates() {
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", &x1_);
}
//}

///* get_all_computed_states() //{ */
void NmpcController::getAllControlInputs() {
  double u[NU];
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &u);

    std::cout << "N = " << i << std::endl;
    for (auto j = 0; j < NU; j++) {
      std::cout << u[j] << std::endl;
    }
    std::cout << std::endl << std::endl;
    /* output.push_back(utraj); */
  }
  /* return output; */
}
//}

///* getCorrection() //{ */
laser_msgs::msg::AttitudeRatesAndThrust NmpcController::getCorrection(geometry_msgs::msg::Pose reference, const nav_msgs::msg::Odometry msg) {
  // --- Update reference
  yref_[x]   = reference.position.x;  // x
  yref_[y]   = reference.position.y;  // y
  yref_[z]   = reference.position.z;  // z
  yref_[qw]  = 1.00;                  // qw
  yref_[qx]  = 0.00;                  // qx
  yref_[qy]  = 0.00;                  // qy
  yref_[qz]  = 0.00;                  // qz
  yref_[vbx] = 0.00;                  // vbx
  yref_[vby] = 0.00;                  // vby
  yref_[vbz] = 0.00;                  // vbz
  yref_[wx]  = 0.00;                  // wx
  yref_[wy]  = 0.00;                  // wy
  yref_[wz]  = 0.00;                  // wz

  u0_[0] = hover_thrust_;
  u0_[1] = hover_thrust_;
  u0_[2] = hover_thrust_;
  u0_[3] = hover_thrust_;

  // --- Read Estimate
  x0_[x] = msg.pose.pose.position.x;
  x0_[y] = msg.pose.pose.position.y;
  x0_[z] = msg.pose.pose.position.z;

  x0_[qw] = msg.pose.pose.orientation.w;
  x0_[qx] = msg.pose.pose.orientation.x;
  x0_[qy] = msg.pose.pose.orientation.y;
  x0_[qz] = msg.pose.pose.orientation.z;

  x0_[vbx] = msg.twist.twist.linear.x;
  x0_[vby] = msg.twist.twist.linear.y;
  x0_[vbz] = msg.twist.twist.linear.z;

  x0_[wx] = msg.twist.twist.angular.x;
  x0_[wy] = msg.twist.twist.angular.y;
  x0_[wz] = msg.twist.twist.angular.z;

  setInitState();
  setInitSolution();
  setReference();
  ocpSolver();
  printStats();
  getFirstControlInput();
  getFirstComputedStates();
  getAllComputedStates();
  /* getAllControlInputs(); */

  double elapsed_time;
  ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);

  laser_msgs::msg::AttitudeRatesAndThrust input_control;
  input_control.roll_rate  = x1_[wx];
  input_control.pitch_rate = x1_[wy];
  input_control.yaw_rate   = x1_[wz];
  /* input_control.total_thrust_normalized = sqrt((u0_[0] + u0_[1] + u0_[2] + u0_[3])) / (62.8); // 62.8 e a soma dos 4 motores produzindo thrust em 85% isso
   * para o x500 */

  // parametros para modelar curva de throtle retirados do mrs para o x500
  double a = 0.27665;
  double b = -0.19642;
  /* double T = sqrt((u0_[0] + u0_[1] + u0_[2] + u0_[3]) / 62.8); */
  /* double T = (u0_[0] + u0_[1] + u0_[2] + u0_[3]) / 62.8; */
    /* double T = sqrt((u0_[0] + u0_[1] + u0_[2] + u0_[3]) / 44); */
  double T = (a * sqrt(((u0_[0] + u0_[1] + u0_[2] + u0_[3])/4))) + b;
  /* double T0 = (a * sqrt((u0_[0]) / GRAVITY)) + b; */
  /* double T1 = (a * sqrt((u0_[1]) / GRAVITY)) + b; */
  /* double T2 = (a * sqrt((u0_[2]) / GRAVITY)) + b; */
  /* double T3 = (a * sqrt((u0_[3]) / GRAVITY)) + b; */
  /* double T = T0 + T1 + T2 + T3; */
  input_control.total_thrust_normalized = T;
  /* input_control.total_thrust_normalized = */
  /*     ((a * sqrt(u0_[0] + u0_[1] + u0_[2] + u0_[3])) + b) > 1 */
  /*         ? 1 */
  /*         : (a * sqrt(u0_[0] + u0_[1] + u0_[2] + u0_[3])) + b;  // 62.8 e a soma dos 4 motores produzindo thrust em 85% isso para o x500 */

  std::cout << "First Control Input" << std::endl;
  std::cout << "elapsed_time   " << elapsed_time << std::endl;
  std::cout << "w1 " << u0_[0] << std::endl;
  std::cout << "w2 " << u0_[1] << std::endl;
  std::cout << "w3 " << u0_[2] << std::endl;
  std::cout << "w4 " << u0_[3] << std::endl;

  return input_control;
}
//}
}  // namespace laser_uav_controllers
