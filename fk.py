import numpy as np

def fk(n, theta, alpha, a, d):
    """
    Compute the forward kinematics transformation matrix for a robotic manipulator.

    Parameters:
    n (int): Number of links in the manipulator.
    theta (list): List of joint angles.
    alpha (list): List of link twists.
    a (list): List of link lengths.
    d (list): List of link offsets.

    Returns:
    result_transform (numpy.ndarray): The final transformation matrix.
    """
    # Initialize the result transformation matrix as an identity matrix
    result_transform = np.eye(4)

    # Iterate through each link
    for i in range(n):
        # Compute the transformation matrix for the current link
        trans_mat = np.array([
            [np.cos(theta[i]), -np.sin(theta[i]) * np.cos(alpha[i]), np.sin(theta[i]) * np.sin(alpha[i]), a[i] * np.cos(theta[i])],
            [np.sin(theta[i]), np.cos(theta[i]) * np.cos(alpha[i]), -np.cos(theta[i]) * np.sin(alpha[i]), a[i] * np.sin(theta[i])],
            [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
            [0, 0, 0, 1]
        ])

        # Multiply the current transformation matrix with the result
        result_transform = np.dot(result_transform, trans_mat)

    return result_transform