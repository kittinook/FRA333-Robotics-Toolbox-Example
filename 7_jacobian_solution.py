import numpy as np
import matplotlib.pyplot as plt
import time
import math

def compute_manipulability(J):
    """Compute the manipulability for a given Jacobian matrix."""
    w = np.sqrt(np.linalg.det(J @ J.T))
    return w
# Define robot parameters
a1, a2 = 1.5, 1  # Link lengths

# Joint configuration
q = np.array([np.pi/4, np.pi])

# Joint velocities
q_dot = [0.0, 0.0]

# Compute end-effector position using forward kinematics
x = a1 * np.cos(q[0]) + a2 * np.cos(q[0] + q[1])
y = a1 * np.sin(q[0]) + a2 * np.sin(q[0] + q[1])

# Compute the Jacobian
J = np.array([
    [-a1 * np.sin(q[0]) - a2 * np.sin(q[0] + q[1]), -a2 * np.sin(q[0] + q[1])],
    [a1 * np.cos(q[0]) + a2 * np.cos(q[0] + q[1]), a2 * np.cos(q[0] + q[1])]
])

# Calculate end-effector velocities
v = J @ q_dot
t_1 = time.time()
dt = 0.01
x_g = 0.5
y_g = 0.0
k_p = 4

fig, ax = plt.subplots()
while True:
    if time.time() - t_1 > dt:
        t_1 = time.time()
        # Compute end-effector position using forward kinematics
        # q1 = q[0]
        # q2 = q[1]
        x = a1 * math.cos(q[0]) + a2 * math.cos(q[0] + q[1])
        y = a1 * math.sin(q[0]) + a2 * math.sin(q[0] + q[1])
        J = np.array([
            [-a1 * np.sin(q[0]) - a2 * np.sin(q[0] + q[1]), -a2 * np.sin(q[0] + q[1])],
            [a1 * np.cos(q[0]) + a2 * np.cos(q[0] + q[1]), a2 * np.cos(q[0] + q[1])]
        ])
        J_1 = J[:, 0]
        J_2 = J[:, 1]

        e_x = x_g - x
        e_y = y_g - y
        e = np.array([[e_x], [e_y]])
        v = e * k_p
        q_dot = np.linalg.inv(J) @ v
        
        q[0] = q[0] + (q_dot[0] * dt)
        q[1] = q[1] + (q_dot[1] * dt)
        
        ax.clear()
        ax.plot([0, a1 * np.cos(q[0]), x], [0, a1 * np.sin(q[0]), y], 'o-')  # Plot robot
        ax.quiver(x, y, J_1[0], J_1[1], color='r')  # Plot velocity vector
        ax.quiver(x, y, J_2[0], J_2[1], color='b')  # Plot velocity vector
        ax.set_xlim([-a1-a2, a1+a2])
        ax.set_ylim([-a1-a2, a1+a2])
        plt.grid(True)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("2R Robot with End-effector Velocity")
        m = compute_manipulability(J)
        
        print(m)
        plt.pause(0.1)
        # q[1] = q[1] + (q_dot*dt)
         