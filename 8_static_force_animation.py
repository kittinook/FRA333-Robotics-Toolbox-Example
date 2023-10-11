import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

L1 = 1.0
L2 = 1.0

theta1 = np.pi / 4  # Example joint value
theta2 = np.pi / 4  # Example joint value

def jacobian(theta1, theta2):
    j11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    j12 = -L2 * np.sin(theta1 + theta2)
    j21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    j22 = L2 * np.cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])

def forward_kinematics(theta1, theta2):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

def compute_ellipse(matrix, scale=1.0):
    eigenvalues, eigenvectors = np.linalg.eig(matrix)
    axis_lengths = scale * np.sqrt(eigenvalues)

    theta = np.linspace(0, 2 * np.pi, 100)
    x = axis_lengths[0] * np.cos(theta)
    y = axis_lengths[1] * np.sin(theta)

    R = eigenvectors
    return R @ np.array([x, y])

fig, ax = plt.subplots()
ax.set_xlim(-2*L1, 2*L1)
ax.set_ylim(-2*L1, 2*L1)
ax.set_aspect('equal')
ax.grid(True)

# Initial configuration
theta1 = 0
theta2 = 0

lines = {
    'robot': ax.plot([], [], '-o', color='blue', label='Robot')[0],
    'velocity_ellipse': ax.plot([], [], color='red', label='Velocity Ellipsoid')[0],
    'force_ellipse': ax.plot([], [], color='green', label='Force Ellipsoid')[0]
}

def init():
    lines['robot'].set_data([], [])
    lines['velocity_ellipse'].set_data([], [])
    lines['force_ellipse'].set_data([], [])
    return lines.values()

def update(frame):
    theta1 = frame * np.pi / 180
    theta2 = -frame * np.pi / 180

    x_end, y_end = forward_kinematics(theta1, theta2)
    
    J = jacobian(theta1, theta2)
    velocity_ellipse = compute_ellipse(J @ J.T)
    force_ellipse = compute_ellipse(np.linalg.pinv(J).T @ np.linalg.pinv(J), scale=0.3)
    
    lines['robot'].set_data([0, L1 * np.cos(theta1), x_end], [0, L1 * np.sin(theta1), y_end])
    lines['velocity_ellipse'].set_data(velocity_ellipse[0, :] + x_end, velocity_ellipse[1, :] + y_end)
    lines['force_ellipse'].set_data(force_ellipse[0, :] + x_end, force_ellipse[1, :] + y_end)

    return lines.values()

ani = FuncAnimation(fig, update, frames=np.linspace(0, 180, 180), init_func=init, blit=True)

plt.legend()
plt.show()