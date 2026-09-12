import math
import numpy as np
from rx150_goal_control.URDF_params import URDF_Params
def inverse_kinematics_newton(init_q, target, max_iter=100):
    q = np.array(init_q, dtype=float)

    for i in range(max_iter):
        x, y, z = forward_kinematics(*q)

        error = np.array([
            target[0] - x,
            target[1] - y,
            target[2] - z,
        ])

        if np.linalg.norm(error) < 0.01:
            return q

        J = jacobian(q)   #雅可比矩阵在误差修正输入的时候很有用

        delta_q = np.linalg.pinv(J) @ error   #重要公式e = j @ \delta q 

        q = q + delta_q

    

    return None

def jacobian( q, delta=1e-5):  #delta设成极小值硬核的求导
    J = np.zeros((3, len(q)))

    x0, y0, z0 = forward_kinematics(*q)

    for j in range(len(q)):
        q_plus = q.copy()
        q_plus[j] += delta

        x1, y1, z1 = forward_kinematics(*q_plus,params=URDF_Params)

        J[0, j] = (x1 - x0) / delta
        J[1, j] = (y1 - y0) / delta
        J[2, j] = (z1 - z0) / delta

    return J

def forward_kinematics( waist, shoulder,elbow,wrist_angle,wrist_rotate,params = URDF_Params):
    T_shoulder = np.array([
        [math.cos(shoulder),0,math.sin(shoulder),0],
        [0,1,0,0],
        [-math.sin(shoulder),0,math.cos(shoulder),params.shoulder_z],
        [0,0,0,1]
    ])

    T_waist = np.array([
        [math.cos(waist),-math.sin(waist),0,0],
        [math.sin(waist),math.cos(waist),0,0],
        [0,0,1,params.waist_z],
        [0,0,0,1]
    ])

    T_elbow = np.array([
        [math.cos(elbow),0,math.sin(elbow),params.elbow_x],
        [0.0,1,0.0,0],
        [-math.sin(elbow),0,math.cos(elbow),params.elbow_z],
        [0,0,0,1]
    ])

    T_wrist_angle = np.array([
        [math.cos(wrist_angle),0,math.sin(wrist_angle),params.wrist_angle_x],
        [0,1,0,0],
        [-math.sin(wrist_angle),0,math.cos(wrist_angle),params.wrist_angle_z],
        [0,0,0,1]
    ])

    T_wrist_rotate = np.array([
        [1,0,0,params.wrist_rotate_x],
        [0,math.cos(wrist_rotate),-math.sin(wrist_rotate),0],
        [0,math.sin(wrist_rotate),math.cos(wrist_rotate),0],
        [0,0,0,1]
    ])
    
    p_local = np.array([
        [0],
        [0],
        [0],
        [1],
    ])

    p_mat = T_waist @ T_shoulder @ T_elbow @ T_wrist_angle @ T_wrist_rotate @ p_local #

    x = p_mat[0,0]
    y = p_mat[1,0]
    z = p_mat[2,0]
    return x,y,z