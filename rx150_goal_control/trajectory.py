import numpy as np

def cubic_trajectory(q_start, q_goal, duration, steps):
    t = np.linspace(0, duration, steps)

    q_start = np.array(q_start, dtype=float)
    q_goal = np.array(q_goal, dtype=float)

    a0 = q_start
    a1 = np.zeros_like(q_start) #生成和q_start元素结构和排布一样，但数值全是0的数组
    a2 = 3.0 * (q_goal - q_start) / (duration ** 2)
    a3 = -2.0 * (q_goal - q_start) / (duration ** 3)

    traj = []

    for time in t:
        q = a0 + a1 * time + a2 * time**2 + a3 * time**3
        traj.append(q)

    return traj