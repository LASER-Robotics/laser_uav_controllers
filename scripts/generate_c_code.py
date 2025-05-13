from acados_template import AcadosOcp, AcadosOcpSolver
from quadrotor_model import export_quadrotor_ode_model
import numpy as np
import scipy.linalg
import os

# create ocp object to formulate the OCP
ocp = AcadosOcp()

# set model
model = export_quadrotor_ode_model()
ocp.model = model

Tf = 1
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx
N = 20

# set dimensions
q_ref_init = np.array([1.0, 0.0, 0.0, 0.0])
inertia = [0.001, 0.001, 0.0014]
C_drag = [0.0, 0.0, 0.0]
C_tau = 0.05
mass = 0.85
motor_pos_0 = [0.15, -0.15]
motor_pos_1 = [-0.15, 0.15]
motor_pos_2 = [-0.15, -0.15]
motor_pos_3 = [0.15, 0.15]
ocp.parameter_values = np.array([mass, motor_pos_0[0], motor_pos_0[1], motor_pos_1[0], motor_pos_1[1], motor_pos_2[0], motor_pos_2[1], motor_pos_3[0], motor_pos_3[1],  inertia[0], inertia[1], inertia[2], C_tau, C_drag[0], C_drag[1], C_drag[2], q_ref_init[0], q_ref_init[1], q_ref_init[2], q_ref_init[3]])

# Constraints
omega_max = np.array([10.0, 10.0, 4.0])  # [rad/s]
thrust_min = 0.0       # [N]
thrust_max = 15.0       # [N] per motor
thrust_total_max = 68.0 # [N] total thrust
thrust_total_min = 0.0 # [N] total thrust

# set constraints
Jbx = np.zeros((3, nx)) # matrix for assigning constrain matrix "omega_max" to corresponding state variables
Jbx[0, 10] = 1.0
Jbx[1, 11] = 1.0
Jbx[2, 12] = 1.0
ocp.constraints.Jbx = Jbx
ocp.constraints.lbx = -1*omega_max
ocp.constraints.ubx = omega_max

Jbu = np.identity(nu) # matrix for assigning thrust constrain matrix to corresponding input variables
ocp.constraints.Jbu = Jbu
ocp.constraints.lbu = thrust_min * np.ones((nu,))
ocp.constraints.ubu = thrust_max * np.ones((nu,))
ocp.constraints.D = np.ones((1, nu))
ocp.constraints.C = np.zeros((1, nx))
ocp.constraints.ug = np.ones((1)) * thrust_total_max
ocp.constraints.lg = np.ones((1)) * thrust_total_min

init_state = np.zeros([nx])
init_state[3] = 1 # quaternion must be unitary
ocp.constraints.x0 = init_state

# set cOSt
Q_p_xy = 0.5
Q_p_z = 1
# Q_p = yaml_parse(data, "Q_position", 100)
Q_r_xy = 0.2
Q_r_z = 3
Q_v = 0.2
Q_o = 0.5
R_t = 3
Q_mat = np.diag([Q_p_xy, Q_p_xy, Q_p_z, Q_r_xy, Q_r_xy, Q_r_z,
                 Q_v, Q_v, Q_v, Q_o, Q_o, Q_o])
R_mat = np.diag([R_t for i in range(nu)])

ny = nx + nu
ny_e = nx

# set cost
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.cost.cost_type_e = 'NONLINEAR_LS'
W_mat = scipy.linalg.block_diag(Q_mat, R_mat)
ocp.cost.W = W_mat
ocp.cost.W_e = Q_mat

# initial references
hover_prop = mass * 9.8066 / 4.0
ocp.cost.yref = np.array(
    [0, 0, 3,\
    0, 0, 0, \
    0, 0, 0, \
    0, 0, 0, \
    hover_prop, hover_prop, hover_prop, hover_prop])
ocp.cost.yref_e = np.array([0, 0, 3,\
                               0, 0, 0,\
                               0, 0, 0,\
                               0, 0, 0])

# set options
ocp.solver_options.N_horizon = N
ocp.solver_options.tf = Tf # set prediction horizon
ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "PARTIAL_CONDENSING_HPIPM", "FULL_CONDENSING_HPIPM"
ocp.solver_options.nlp_solver_type = "SQP_RTI"     # "SQP", "SQP_RTI"
ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # "GAUSS_NEWTON", "EXACT"
ocp.solver_options.integrator_type = "ERK"   # "ERK", "IRK", "GNSF"
ocp.solver_options.hpipm_mode = "SPEED_ABS"
#ocp.solver_options.sim_method_jac_reuse = True
#ocp.solver_options.sim_method_newton_iter = 1
#ocp.solver_options.sim_method_num_stages = 3
#ocp.solver_options.sim_method_num_steps = 1
# ocp.solver_options.print_level = 1

dir_path = "."

ocp.code_export_directory = dir_path + "/c_generated_code"

# this will generate c code
ocp_solver = AcadosOcpSolver(ocp, json_file = f'{dir_path}/acados_ocp.json')
 
# this will do simulation also in python
 
# status = ocp_solver.solve()
# ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")
# 
# if status != 0:
#     raise Exception(f'acados returned status {status}.')
# 
# # get solution
# simX = np.ndarray((N+1, nx))
# simU = np.ndarray((N, nu))
# for i in range(N):
#     simX[i,:] = ocp_solver.get(i, "x")
#     simU[i,:] = ocp_solver.get(i, "u")
# simX[N,:] = ocp_solver.get(N, "x")
# 
# plot_quadrotor(np.linspace(0, Tf, N+1), thrust_max, simU, simX, latexify=False)
