from acados_template import AcadosModel
from casadi import DM, MX, SX, vertcat, sin, cos, Function, inv, cross, mtimes, diag, sqrt, norm_2, reshape, dot
import numpy as np

def quaternion_multiplication(q1,q2):
    ans = vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
           q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
           q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
           q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

def rotate_quaternion(q1,v1):
    ans = quaternion_multiplication(quaternion_multiplication(q1, vertcat(0, v1)), vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec

def quaternion_error(q, q_ref):
    q_aux = vertcat(q[0, :] * q_ref[0, :] + q[1, :] * q_ref[1, :] + q[2, :] * q_ref[2, :] + q[3, :] * q_ref[3, :], 
                    - q[1, :] * q_ref[0, :] + q[0, :] * q_ref[1, :] + q[3, :] * q_ref[2, :] - q[2, :] * q_ref[3, :], 
                    - q[2, :] * q_ref[0, :] - q[3, :] * q_ref[1, :] + q[0, :] * q_ref[2, :] + q[1, :] * q_ref[3, :], 
                    - q[3, :] * q_ref[0, :] + q[2, :] * q_ref[1, :] - q[1, :] * q_ref[2, :] + q[0, :] * q_ref[3, :])
    
    # attitude errors. SQRT have small quantities added (1e-3) to alleviate the derivative
    # not being defined at zero, and also because it's in the denominator
    q_att_denom = sqrt(q_aux[0] * q_aux[0] + q_aux[3] * q_aux[3] + 1e-3)
    q_att = (
        vertcat(
            q_aux[0] *
            q_aux[1] -
            q_aux[2] *
            q_aux[3],
            q_aux[0] *
            q_aux[2] +
            q_aux[1] *
            q_aux[3],
            q_aux[3],
        ) /
        q_att_denom)
    return q_att

def export_multirotor_ode_model() -> AcadosModel:
    model_name = "multirotor_ode"

    # Mass constant
    m = SX.sym("m")

    # Inertial Matrix and your inverse
    I_diag = SX.sym("I_diag", 3)
    I = diag(I_diag)
    I_inv = diag(1 / I_diag)

    # Coeficient of drag in axis and coeficient of torque
    C_tau = SX.sym("C_tau")
    C_drag = SX.sym("C_drag", 3)

    quaternion_ref = SX.sym("quaternion_ref", 4);

    # motor's G1 Matrix (Allocation Matrix)
    G1 = SX.sym('G1', 8, 4)

    # x
    p = SX.sym('p', 3)              # position
    q = SX.sym('q', 4)              # quaternion
    v = SX.sym('v', 3)              # linear velocity
    w = SX.sym('w', 3)              # angular velocity
    x = vertcat(p, q, v, w)         # system's states definition

    # u
    T = SX.sym('thrust', 8)         # individual thrusts
    u = vertcat(T)                  # system's input definition

    # xdot
    p_dot = SX.sym('p_dot', 3)      # position derivation
    q_dot = SX.sym('q_dot', 4)      # quaternion derivation
    v_dot = SX.sym('v_dot', 3)      # linear velocity derivation
    w_dot = SX.sym('w_dot', 3)      # angular velocity derivation
    xdot = vertcat(p_dot, q_dot, v_dot, w_dot) # system's states derivation

    MAX_OBSTACLES = 5
    
    # 1. Criação das variáveis simbólicas com os novos tamanhos
    A_m = SX.sym('A_m', 3 * MAX_OBSTACLES) 
    b_m = SX.sym('b_m', MAX_OBSTACLES)
    
    # 2. Criação da equação de restrição h
    h_list = [] # Usamos uma lista normal do Python
    
    # Fazemos um loop para montar uma equação h para cada obstáculo
    for i in range(MAX_OBSTACLES):
        # Puxa o vetor 3D e o escalar correspondentes ao obstáculo 'i'
        A_i = A_m[i*3 : (i+1)*3]  # Pega de 3 em 3
        b_i = b_m[i]
        
        # Equação clássica do RVC para este obstáculo (usando 'v')
        h_i = dot(A_i, v) - b_i
        
        # Guarda na lista
        h_list.append(h_i)
        
    # Concatena a lista toda de uma vez só (Forma 100% segura no CasADi)
    h_rvc = vertcat(*h_list) 

    # ADICIONAMOS NO VETOR 'par' (Isso vai mudar o tamanho dele de 43 para 55!)
    # Parameters definition
    par = vertcat(m, reshape(G1, -1, 1), I_diag, C_drag, quaternion_ref, A_m, b_m)

    g_ = 9.806
    g = SX([0, 0, -g_])             # gravity acceleration

    a_drag = SX.zeros(3)

    q_normalized = q/norm_2(q)

    wrench = mtimes(G1.T, T)
    tau = vertcat(wrench[1:])
    total_acceleration = wrench[0] / m

    # Derivate States
    dot_p = v # v
    dot_q = 0.5 * quaternion_multiplication(q_normalized, vertcat(0, w)) # 1/2 * q @ [0, wx, wy, wz]
    dot_v = rotate_quaternion(q_normalized, vertcat(0, 0, total_acceleration)) + g - a_drag # q @ [0, 0, T] + [0, 0, -g] - v_cd
    dot_w = mtimes(I_inv, tau - cross( w, mtimes(I, w))) # I_inv * (AT - w X Iw)

    f_expl = vertcat(dot_p, dot_q, dot_v, dot_w)
    f_impl = xdot - f_expl
    
    # 3. CRIAÇÃO DO MODELO E ATRIBUIÇÃO
    model = AcadosModel()
   
    model.con_h_expr = h_rvc
    
    model.f_impl_expr = f_impl # explicit dynamics
    model.f_expl_expr = f_expl # implicit dynamics
    
    q_att = quaternion_error(q_normalized, quaternion_ref)
    model.cost_y_expr = vertcat(p, q_att, v, w, u)
    model.cost_y_expr_e = vertcat(p, q_att, v, w)
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = par
    model.name = model_name

    return model
