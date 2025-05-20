from acados_template import AcadosModel
from casadi import DM, MX, SX, vertcat, sin, cos, Function, inv, cross, mtimes, diag, sqrt, norm_2
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

def export_quadrotor_ode_model() -> AcadosModel:
    model_name = "quadrotor_ode"

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

    # motor's distance to mass center
    motor_pos_0 = SX.sym('motor_pos_0',2)
    motor_pos_1 = SX.sym('motor_pos_1',2)
    motor_pos_2 = SX.sym('motor_pos_2',2)
    motor_pos_3 = SX.sym('motor_pos_3',2)

    # x
    p = SX.sym('p', 3)              # position
    q = SX.sym('q', 4)              # quaternion
    v = SX.sym('v', 3)              # linear velocity
    w = SX.sym('w', 3)              # angular velocity
    x = vertcat(p, q, v, w)         # system's states definition

    # u
    T = SX.sym('thrust', 4)         # individual thrusts
    u = vertcat(T)                  # system's input definition

    # xdot
    p_dot = SX.sym('p_dot', 3)      # position derivation
    q_dot = SX.sym('q_dot', 4)      # quaternion derivation
    v_dot = SX.sym('v_dot', 3)      # linear velocity derivation
    w_dot = SX.sym('w_dot', 3)      # angular velocity derivation
    xdot = vertcat(p_dot, q_dot, v_dot, w_dot) # system's states derivation

    # Parameters definition
    par = vertcat(m, motor_pos_0, motor_pos_1, motor_pos_2, motor_pos_3, I_diag, C_tau, C_drag, quaternion_ref)

    g_ = 9.806
    g = SX([0, 0, -g_])             # gravity acceleration

    a_drag = SX.zeros(3)

    q_normalized = q/norm_2(q)
    # q_conj = vertcat(q_normalized[0], -q_normalized[1], -q_normalized[2], -q_normalized[3])
    # # q_conj = q_conj/norm_1(q_conj)
    # print(q_conj)
    # v_body = rotate_quaternion(q_conj, v)
    # print(v_body)
    # f_drag_body = (C_drag/m) * v_body
    # print(f_drag_body)
    # a_drag = rotate_quaternion(q_normalized, f_drag_body)

    total_thrust = (T[0] + T[1] + T[2] + T[3])/m

    # Derivate States
    dot_p = v # v
    # dot_p = rotate_quaternion(q_normalized, v) # v
    dot_q = 0.5 * quaternion_multiplication(q_normalized, vertcat(0, w)) # 1/2 * q @ [0, wx, wy, wz]
    dot_v = rotate_quaternion(q_normalized, vertcat(0, 0, total_thrust)) + g - a_drag # q @ [0, 0, T] + [0, 0, -g] - v_cd
    dot_w = mtimes(I_inv, vertcat((+ T[0]*motor_pos_0[1] + T[1]*motor_pos_1[1] + T[2]*motor_pos_2[1] + T[3]*motor_pos_3[1]),
                                  (- T[0]*motor_pos_0[0] - T[1]*motor_pos_1[0] - T[2]*motor_pos_2[0] - T[3]*motor_pos_3[0]),
                                  C_tau*(-T[0]+T[1]+T[2]-T[3])) - cross( w, mtimes(I, w))) # I_inv * (AT - w X Iw)

    f_expl = vertcat(dot_p, dot_q, dot_v, dot_w)

    f_impl = xdot - f_expl

    model = AcadosModel()
    print(type(f_expl))
    print(isinstance(f_expl, SX))
    model.f_impl_expr = f_impl # explicit dynamics
    model.f_expl_expr = f_expl # implicit dynamics
    q_att = quaternion_error(q_normalized, quaternion_ref)
    model.cost_y_expr = vertcat(p, q_att, v, w, u)
    model.cost_y_expr_e = vertcat(p, q_att, v, w)
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = par
    model.name = model_name

    return model
