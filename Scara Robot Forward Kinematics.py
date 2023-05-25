import numpy as np

def scara_forward_kinematics(theta1, theta2, d3, theta4):
    # DH parameters
    a1 = 1.0  # Length of link 1
    a2 = 1.0  # Length of link 2
    d1 = 0.5  # Offset between base and link 1
    d2 = 0.0  # Offset between link 1 and link 2
    d4 = 0.2  # Offset between link 3 and end effector

    # Joint angles
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    theta4 = np.radians(theta4)

    # Homogeneous transformation matrices
    T0_1 = np.array([
        [np.cos(theta1), -np.sin(theta1), 0, a1 * np.cos(theta1)],
        [np.sin(theta1), np.cos(theta1), 0, a1 * np.sin(theta1)],
        [0, 0, 1, d1],
        [0, 0, 0, 1]
    ])

    T1_2 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0, a2 * np.cos(theta2)],
        [np.sin(theta2), np.cos(theta2), 0, a2 * np.sin(theta2)],
        [0, 0, 1, d2],
        [0, 0, 0, 1]
    ])

    T2_3 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d3],
        [0, 0, 0, 1]
    ])

    T3_4 = np.array([
        [np.cos(theta4), -np.sin(theta4), 0, 0],
        [np.sin(theta4), np.cos(theta4), 0, 0],
        [0, 0, 1, d4],
        [0, 0, 0, 1]
    ])

    # Calculate the transformation matrix of the end effector
    T0_4 = T0_1 @ T1_2 @ T2_3 @ T3_4

    # Extract the position and orientation from the transformation matrix
    position = T0_4[:3, 3]
    orientation = T0_4[:3, :3]

    return position, orientation

# Example usage
theta1 = 45.0  # Joint 1 angle in degrees
theta2 = 30.0  # Joint 2 angle in degrees
d3 = 0.4  # Joint 3 offset
theta4 = -15.0  # Joint 4 angle in degrees

position, orientation = scara_forward_kinematics(theta1, theta2, d3, theta4)

print("End Effector Position:", position)
print("End Effector Orientation:")
print(orientation)
