import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import time




L1 = 1.0
L2 = 1.0
t_1 = 0
m1 = 0.5
m2 = 0.5
g = 9.81

def jacobian(theta1, theta2):
    j11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    j12 = -L2 * np.sin(theta1 + theta2)
    j21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    j22 = L2 * np.cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])

def forward_kinematics(theta1, theta2):
    x = L1 * np.sin(theta1) + L2 * np.sin(theta2)
    y = -L1 * np.cos(theta1) - L2 * np.cos(theta2)
    return x, y


def Mq(q):
    q1 = q[0]
    q2 = q[1]
    return np.array( [
        [(m1+m2)*L1*L1, (m1*L1*L2)*math.cos(q1-q2)],
        [m2*L1*L2*math.cos(q1-q2), m2*L2*L2]
    ] )

def Bq_qd(q, qd):
    q1 = q[0]
    q2 = q[1]
    qd_1 = qd[0]
    qd_2 = qd[1]
    return np.array( [
        [m2*L1*L2*qd_2*qd_2*math.sin(q1-q2)],
        [m2*L1*L2*qd_1*qd_1*math.sin(q1-q2)]
    ])
    
def gq(q):
    q1 = q[0]
    q2 = q[1]
    return np.array( [
        [(m1+m2)*g*L1*math.sin(q1)],
        [m2*g*L2*math.sin(q2)]
    ])
    
def damp(qd, b):
    qd_1 = qd[0]
    qd_2 = qd[1]
    return np.array( [
        [b * qd_1],
        [b * qd_2]
    ])

q_sim = np.array([0.9,0.3])
qd_sim = np.array([0.0, 0.0])

dt = 0.1
fig, ax = plt.subplots()

ax.grid(True)

while True:
    if time.time() - t_1 >= dt:
        t_1 = time.time()
        x, y = forward_kinematics(q_sim[0], q_sim[1])
        # qdd = M(q)^-1 * (T - JtFc - B(q,qd) - G(q))
        qdd_sim = np.linalg.inv( Mq(q_sim) ) @ ( -damp(qd_sim, 2.1) - Bq_qd(q_sim, qd_sim) - gq(q_sim) )
        qd_sim[0] = qd_sim[0] + (qdd_sim[0] * dt)
        qd_sim[1] = qd_sim[1] + (qdd_sim[1] * dt)
        q_sim[0] = q_sim[0] + (qd_sim[0] * dt)
        q_sim[1] = q_sim[1] + (qd_sim[1] * dt)
        
        # print(q_sim)
        ax.clear()
        ax.plot([0, L1 * math.sin(q_sim[0]),L1 * math.sin(q_sim[0]) + L2 * math.sin(q_sim[1])], 
                [0, -L1 * math.cos(q_sim[0]), -L1 * math.cos(q_sim[0]) - L2 * math.cos(q_sim[1])], 'o-')  # Plot robot

        ax.set_xlim(-2*L1, 2*L1)
        ax.set_ylim(-2*L1, 2*L1)
        ax.set_aspect('equal')
        plt.pause(dt)