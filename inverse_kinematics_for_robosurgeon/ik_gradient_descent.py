import numpy as np
import forward_kinematics

def inverse_kinematics_GD(desired_pose, n, alpha, a, d):
    # Convert degrees to radians
    alpha = np.deg2rad(alpha)
    
    # Initialize joint angles
    joint_angles = np.zeros(n)
    
    # Parameters for the inverse kinematics algorithm
    max_iterations = 300000
    tolerance = 1e-6
    learning_rate = 0.001
    epsilon = 1e-6
    stagnation_window = 10000
    count = 0
    
    # Initialize error history
    error_history = []
    
    # Compute the initial pose using forward kinematics
    current_pose = forward_kinematics.fk(n, joint_angles, alpha, a, d)
    
    for iter in range(max_iterations):
        # Compute the error between the desired pose and the current pose
        error = np.linalg.norm(desired_pose - current_pose[:3, 3])
        error_history.append(error)
        
        # Check for stagnation
        if iter >= 1 and round(error_history[iter], 6) == round(error_history[iter - 1], 6):
            count += 1
        else:
            count = 0
        
        # Check for convergence or stagnation
        if error <= tolerance or count == stagnation_window:
            print("Inverse kinematics converged")
            break
        
        # Update the Jacobian matrix
        J = np.zeros((3, n))
        for i in range(n):
            # Increment the current joint angle
            joint_angles_incremented = joint_angles.copy()
            joint_angles_incremented[i] += epsilon
            
            # Compute the new pose with the incremented joint angle
            new_pose = forward_kinematics.fk(n, joint_angles_incremented, alpha, a, d)
            
            # Compute the change in pose
            delta_pose = new_pose[:3, 3] - current_pose[:3, 3]
            J[:, i] = delta_pose / epsilon
        
        # Gradient descent update
        gradient = J.T @ (desired_pose - current_pose[:3, 3])
        joint_angles += learning_rate * gradient
        
        # Update the current pose
        current_pose = forward_kinematics.fk(n, joint_angles, alpha, a, d)
    
    if iter == max_iterations - 1:
        print("Inverse kinematics did not converge within max iterations.")
    
    return joint_angles