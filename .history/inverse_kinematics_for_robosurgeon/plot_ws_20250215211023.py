import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import forward_kinematics  # Ensure this module has the `fk` function

def plot_workspace(n, alpha, a, d, angle_range=(-180, 180), resolution=10):
    """
    Plot the workspace of a robotic manipulator.

    Parameters:
        n (int): Number of joints.
        alpha (np.array): Link twist angles in degrees.
        a (np.array): Link lengths.
        d (np.array): Link offsets.
        angle_range (tuple): Range of joint angles in degrees (min, max).
        resolution (int): Number of samples per joint angle.
    """
    # Generate joint angle combinations
    theta_values = [np.linspace(angle_range[0], angle_range[1], resolution) for _ in range(n)]
    theta_mesh = np.meshgrid(*theta_values)
    theta_combinations = np.vstack([theta_mesh[i].ravel() for i in range(n)]).T

    # Compute end-effector positions
    workspace_points = []
    for theta in theta_combinations:
        T = forward_kinematics.fk(n, theta, alpha, a, d)  # Use the fk function from the module
        workspace_points.append(T[:3, 3])  # Extract the position (x, y, z)

    workspace_points = np.array(workspace_points)

    # Plot the workspace
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(workspace_points[:, 0], workspace_points[:, 1], workspace_points[:, 2], s=1, c='b', alpha=0.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Workspace')
    plt.show()

# Example usage:
n = 7  # Number of links
alpha = np.array([90, 0, 0, 90, -90, -90, 0])  # Link twist angles in degrees
a = np.array([0, 218, 196, 36, 0, 7.5, 11])  # Link lengths
d = np.array([147, 0, 0, 0, 425, 0, 0])  # Link offsets

# Plot the workspace
plot_workspace(n, alpha, a, d, angle_range=(-180, 180), resolution=5)