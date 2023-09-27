import numpy as np
import matplotlib.pyplot as plt

# Define robot parameters
a1, a2 = 1.5, 1  # Link lengths

# Joint configuration
q = [np.pi/4, np.pi/3]

# Joint velocities
q_dot = [0.0, 0.5]

# Compute end-effector position using forward kinematics
x = a1 * np.cos(q[0]) + a2 * np.cos(q[0] + q[1])
y = a1 * np.sin(q[0]) + a2 * np.sin(q[0] + q[1])

# Compute the Jacobian
J = np.array([
    [-a1 * np.sin(q[0]) - a2 * np.sin(q[0] + q[1]), -a2 * np.sin(q[0] + q[1])],
    [a1 * np.cos(q[0]) + a2 * np.cos(q[0] + q[1]), a2 * np.cos(q[0] + q[1])]
])

J_1 = J[:, 0]
J_2 = J[:, 1]
# Calculate end-effector velocities
v = J @ q_dot

# Plot robot and velocities
fig, ax = plt.subplots()
ax.plot([0, a1 * np.cos(q[0]), x], [0, a1 * np.sin(q[0]), y], 'o-')  # Plot robot
ax.quiver(x, y, J_1[0], J_1[1], color='r')  # Plot velocity vector
ax.quiver(x, y, J_2[0], J_2[1], color='b')  # Plot velocity vector
ax.set_xlim([-a1-a2, a1+a2])
ax.set_ylim([-a1-a2, a1+a2])
plt.grid(True)
plt.title("2R Robot with End-effector Velocity")
plt.show()
