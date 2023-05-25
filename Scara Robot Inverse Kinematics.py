import numpy as np

def scara_inverse_kinematics(position, orientation):
    # DH parameters
    a1 = 1.0  # Length of link 1
    a2 = 1.0  # Length of link 2
    d1 = 0.5  # Offset between base and link 1
    d2 = 0.0  # Offset between link 1 and link 2
    d4 = 0.2  # Offset between link 3 and end effector

    # Extract position and orientation components
    px, py, pz = position
    ox, oy, oz = orientation

    # Calculate theta1
    theta1 = np.arctan2(py - d1, px)

    # Calculate theta2
    c2 = (px - a1 * np.cos(theta1))**2 + (py - d1 - a1 * np.sin(theta1))**2 - a2**2 - d2**2
    s2_positive = np.sqrt(1 - c2**2)
    s2_negative = -s2_positive

    theta2_positive = np.arctan2(s2_positive, c2) - np.arctan2(d2, a2)
    theta2_negative = np.arctan2(s2_negative, c2) - np.arctan2(d2, a2)

    # Choose the appropriate solution based on the desired orientation
    if np.isclose(np.sin(theta2_positive), oy) and np.isclose(np.cos(theta2_positive), ox):
        theta2 = theta2_positive
    else:
        theta2 = theta2_negative

    # Calculate theta3
    d3 = pz - d1 - a1 * np.sin(theta1) - a2 * np.sin(theta1 + theta2)

    # Calculate theta4
    theta4 = np.arctan2(oz, np.cos(theta1 + theta2) * ox + np.sin(theta1 + theta2) * oy)

    # Convert the joint angles to degrees
    theta1_deg = np.degrees(theta1)
    theta2_deg = np.degrees(theta2)
    theta4_deg = np.degrees(theta4)

    return theta1_deg, theta2_deg, d3, theta4_deg

# Example usage
position = np.array([1.5, 1.2, 0.8])  # Desired end effector position
orientation = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Desired end effector orientation

theta1, theta2, d3, theta4 = scara_inverse_kinematics(position, orientation)

print("Joint 1 Angle:", theta1)
print("Joint 2 Angle:", theta2)
print("Joint 3 Offset:", d3)
print("Joint 4 Angle:", theta4)
