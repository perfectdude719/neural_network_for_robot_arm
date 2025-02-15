import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import forward_kinematics  # Ensure this module has the `fk` function

def plot_workspace_with_convex_hull(n, alpha, a, d, angle_range=(-180, 180), num_samples=10000):
    """
    Plot the workspace and its convex hull envelope.

    Parameters:
        n (int): Number of joints.
        alpha (np.array): Link twist angles in degrees.
        a (np.array): Link lengths.
        d (np.array): Link offsets.
        angle_range (tuple): Range of joint angles in degrees (min, max).
        num_samples (int): Number of random samples to generate.
    """
    # Generate random joint angle combinations
    theta_combinations = np.random.uniform(low=angle_range[0], high=angle_range[1], size=(num_samples, n))

    # Compute end-effector positions
    workspace_points = []
    for theta in theta_combinations:
        T = forward_kinematics.fk(n, theta, alpha, a, d)
        workspace_points.append(T[:3, 3])  # Extract the position (x, y, z)

    workspace_points = np.array(workspace_points)

    # Compute the convex hull
    hull = ConvexHull(workspace_points)

    # Plot the workspace points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(workspace_points[:, 0], workspace_points[:, 1], workspace_points[:, 2], s=1, c='b', alpha=0.5, label='Workspace Points')

    # Plot the convex hull
    for simplex in hull.simplices:
        ax.plot(workspace_points[simplex, 0], workspace_points[simplex, 1], workspace_points[simplex, 2], 'r-', alpha=0.5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Workspace with Convex Hull Envelope')
    ax.legend()
    plt.show()

# Example usage:
n = 7  # Number of links
alpha = np.array([90, 0, 0, 90, -90, -90, 0])  # Link twist angles in degrees
a = np.array([0, 218, 196, 36, 0, 7.5, 11])  # Link lengths
d = np.array([147, 0, 0, 0, 425, 0, 0])  # Link offsets

# Plot the workspace with convex hull envelope
plot_workspace_with_convex_hull(n, alpha, a, d, angle_range=(-180, 180), num_samples=10000)