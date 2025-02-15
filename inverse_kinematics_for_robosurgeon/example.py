import numpy as np
import forward_kinematics
import ik_gradient_descent
# Example usage:
n =7   # Number of links
theta = np.array([0,0,0,0,0,0,0])  # Joint angles in degrees
alpha = np.array([90,0,0,90,-90,-90,0])  # Link twist angles in degrees
a = np.array([0,218,196,36,0,7.5,11])  # Link lengths
d = np.array([147,0,0,0,425,0,0])  # Link offsets

# Compute forward kinematics
result_transform = forward_kinematics.fk(n, theta, alpha, a, d)
print("Forward Kinematics Result:\n", result_transform)

# Desired pose for inverse kinematics
desired_pose = np.array([1.5, 1.5, 1.5])

# Compute inverse kinematics
joint_angles =ik_gradient_descent.inverse_kinematics_GD(desired_pose, n, alpha, a, d)
print("Inverse Kinematics Result (Joint Angles):\n", joint_angles)