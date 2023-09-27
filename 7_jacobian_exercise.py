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
q = np.array([np.pi/4, np.pi/3])

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
        plt.pause(0.1)
        # q[1] = q[1] + (q_dot*dt)
         