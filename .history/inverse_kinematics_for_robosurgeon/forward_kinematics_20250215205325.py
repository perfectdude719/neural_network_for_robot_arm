import numpy as np

def fk(n, theta, alpha, a, d):
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