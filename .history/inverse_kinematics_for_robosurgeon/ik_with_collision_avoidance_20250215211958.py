import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def fk_for_gui(n, theta, alpha, a, d):
    # Convert degrees to radians
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    
    # Initialize the result transformation matrix as an identity matrix
    result_transform = np.eye(4)
    
    # Iterate through each link to compute the transformation matrix
    for i in range(n):
        # Compute the transformation matrix for the current link
        trans_mat = np.array([
            [np.cos(theta[i]), -np.sin(theta[i]) * np.cos(alpha[i]), np.sin(theta[i]) * np.sin(alpha[i]), a[i] * np.cos(theta[i])],
            [np.sin(theta[i]), np.cos(theta[i]) * np.cos(alpha[i]), -np.cos(theta[i]) * np.sin(alpha[i]), a[i] * np.sin(theta[i])],
            [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
            [0, 0, 0, 1]
        ])
        
        # Multiply the result transformation matrix with the current transformation matrix
        result_transform = np.dot(result_transform, trans_mat)
    
    return result_transform

def check_collision(robot_links, obstacles):
    """
    Check for collisions between robot links and obstacles.
    
    Parameters:
        robot_links (list of np.array): List of link positions (x, y, z).
        obstacles (list of np.array): List of obstacle positions (x, y, z).
    
    Returns:
        bool: True if collision is detected, False otherwise.
    """
    collision_threshold = 0.1  # Minimum distance to avoid collision
    for link in robot_links:
        for obstacle in obstacles:
            distance = np.linalg.norm(link - obstacle)
            if distance < collision_threshold:
                return True  # Collision detected
    return False  # No collision

def inverse_kinematics_with_collision_avoidance(desired_pose, n, alpha, a, d, obstacles, max_iterations=1000, tolerance=1e-6, learning_rate=0.001):
    """
    Inverse Kinematics with collision avoidance.
    
    Parameters:
        desired_pose (np.array): Desired end-effector position (x, y, z).
        n (int): Number of joints.
        alpha (np.array): Link twist angles in degrees.
        a (np.array): Link lengths.
        d (np.array): Link offsets.
        obstacles (list of np.array): List of obstacle positions (x, y, z).
        max_iterations (int): Maximum number of iterations.
        tolerance (float): Convergence tolerance.
        learning_rate (float): Gradient descent step size.
    
    Returns:
        np.array: Joint angles that achieve the desired pose without collisions.
    """
    # Initialize joint angles
    joint_angles = np.zeros(n)
    
    # Gradient descent loop
    for iter in range(max_iterations):
        # Compute forward kinematics
        T = fk_for_gui(n, joint_angles, alpha, a, d)
        current_pose = T[:3, 3]
        
        # Check for collisions
        robot_links = [fk_for_gui(n, joint_angles, alpha, a, d)[:3, 3] for _ in range(n)]  # Simplified link positions
        if check_collision(robot_links, obstacles):
            print("Collision detected! Trying new configuration...")
            joint_angles = np.random.uniform(low=-180, high=180, size=n)  # Randomize joint angles
            continue
        
        # Compute error
        error = desired_pose - current_pose
        if np.linalg.norm(error) < tolerance:
            print("IK converged to a collision-free solution!")
            return joint_angles
        
        # Compute Jacobian numerically
        J = np.zeros((3, n))
        epsilon = 1e-6
        for i in range(n):
            joint_angles_incremented = joint_angles.copy()
            joint_angles_incremented[i] += epsilon
            T_incremented = fk_for_gui(n, joint_angles_incremented, alpha, a, d)
            delta_pose = T_incremented[:3, 3] - current_pose
            J[:, i] = delta_pose / epsilon
        
        # Update joint angles using gradient descent
        joint_angles += learning_rate * J.T @ error
    
    print("IK did not converge to a collision-free solution within max iterations.")
    return None

# Example usage
n = 7  # Number of links
alpha = np.array([90, 0, 0, 90, -90, -90, 0])  # Link twist angles in degrees
a = np.array([0, 218, 196, 36, 0, 7.5, 11])  # Link lengths
d = np.array([147, 0, 0, 0, 425, 0, 0])  # Link offsets

# Define obstacles (x, y, z positions)
obstacles = [np.array([200, 0, 0]), np.array([0, 200, 0])]

# Desired end-effector position
desired_pose = np.array([300, 100, 200])

# Run IK with collision avoidance
joint_angles = inverse_kinematics_with_collision_avoidance(desired_pose, n, alpha, a, d, obstacles)
if joint_angles is not None:
    print("Collision-free joint angles:", joint_angles)