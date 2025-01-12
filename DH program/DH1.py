""" used Chatgpt to get the python file
- only DH value and Joint angle Value is uploaded by me
"""


import numpy as np

# Function to convert degrees to radians
def deg_to_rad(deg):
    return deg * np.pi / 180

# Function to convert radians to degrees
def rad_to_deg(rad):
    return rad * 180 / np.pi

# DH Transformation Matrix
def dh_matrix(theta, d, a, alpha):
    """
    Computes the Denavit-Hartenberg transformation matrix for a single joint.
    theta, alpha in radians, d, a in meters.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward Kinematics for Epson VT6 6-DOF Robot
def forward_kinematics_epson_vt6(joint_angles, dh_params):
    """
    Given joint angles and DH parameters, calculate the end-effector position.
    joint_angles: List of joint angles in degrees [theta1, theta2, ..., theta6]
    dh_params: List of DH parameters as tuples (theta, d, a, alpha) for each joint
    """
    # Initialize the transformation matrix as identity
    T = np.eye(4)
    
    for i in range(6):
        # Convert joint angle from degrees to radians
        theta = deg_to_rad(joint_angles[i])
        # Extract DH parameters for the current joint
        theta_dh, d, a, alpha = dh_params[i]  # Unpack correctly
        # Compute the transformation matrix for the current joint
        T_i = dh_matrix(theta + theta_dh, d, a, alpha)  # Add the joint angle to theta_dh
        # Multiply the transformation matrices
        T = np.dot(T, T_i)
    
    # Extract the position of the end-effector from the final transformation matrix
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    
    # Extract rotation matrix from the transformation matrix (upper-left 3x3 submatrix)
    R = T[:3, :3]
    
    # Calculate Euler angles from the rotation matrix (ZYX convention)
    u = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    v = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))  # Pitch
    w = np.arctan2(R[2, 1], R[2, 2])  # Roll
    
    # Convert Euler angles from radians to degrees
    u_deg = rad_to_deg(u)
    v_deg = rad_to_deg(v)
    w_deg = rad_to_deg(w)
    
    # Convert position from meters to millimeters
    x_mm = x * 1000  # Convert to mm
    y_mm = y * 1000  # Convert to mm
    z_mm = z * 1000  # Convert to mm
    
    return x_mm, y_mm, z_mm, u_deg, v_deg, w_deg

# Example DH parameters for Epson VT6 (in meters and degrees)
# Each joint has four parameters: (theta, d, a, alpha)

dh_params = [
    (-90 , 0.412, 0.1, -90),    # Joint 1: (theta1, d1, a1, alpha1)
    (-90, 0, 0.420, -90),     # Joint 2: (theta2, d2, a2, alpha2)
    (0  , 0, 0, -90),         # Joint 3: (theta3, d3, a3, alpha3)
    (90, 0.40, 0, -90),    # Joint 4: (theta4, d4, a4, alpha4)
    (-90, 0, 0, -90),       # Joint 5: (theta5, d5, a5, alpha5)
    (0, 0.08, 0, 0)     # Joint 6: (theta6, d6, a6, alpha6)
]
"""
dh_params = [
    (-40 , 0.410, 0.11  , 89),    # Joint 1: (theta1, d1, a1, alpha1)
    (-11  , 0.010, 0.390 , 48),     # Joint 2: (theta2, d2, a2, alpha2)
    (-54  , 0.020, 0.310 , 65),         # Joint 3: (theta3, d3, a3, alpha3)
    (-48 , 0.210, 0     , 168),    # Joint 4: (theta4, d4, a4, alpha4)
    (8 , 0.110, 0     , -107),       # Joint 5: (theta5, d5, a5, alpha5)
    (-33  , 0.120, 0     , -25)     # Joint 6: (theta6, d6, a6, alpha6)
]


dh_params = [
    (90 , 0.412, 0, -90),    # Joint 1: (theta1, d1, a1, alpha1)
    (-90, 0, 0.420, 0),     # Joint 2: (theta2, d2, a2, alpha2)
    (0  , 0, 0, -90),         # Joint 3: (theta3, d3, a3, alpha3)
    (180, 0.40, 0, -90),    # Joint 4: (theta4, d4, a4, alpha4)
    (180, 0, 0, -90),       # Joint 5: (theta5, d5, a5, alpha5)
    (180, 0.08, 0, 180)     # Joint 6: (theta6, d6, a6, alpha6)
]
-corrected above

dh_params = [
    (-90 , 0.412 , -0.10, -90),    # Joint 1: (theta1, d1, a1, alpha1)
    (-90 , 0     , 0.420, 0),     # Joint 2: (theta2, d2, a2, alpha2)
    (0   , 0     , 0    , -90),         # Joint 3: (theta3, d3, a3, alpha3)
    (-180, -0.40 , 0    , -90),    # Joint 4: (theta4, d4, a4, alpha4)
    (-180, 0     , 0    , -90),       # Joint 5: (theta5, d5, a5, alpha5)
    (0   , -0.08 , 0    , 180)     # Joint 6: (theta6, d6, a6, alpha6)
]
"""
"""
dh_params = [
    (0  , 0.412  , 0    , 0),    # Joint 1: (theta1, d1, a1, alpha1)
    (-90, 0      , 0.10 , -90),     # Joint 2: (theta2, d2, a2, alpha2)
    (0  , 0      , 0.42 , 0),         # Joint 3: (theta3, d3, a3, alpha3)
    (0  , 0.40   , 0     , -90),    # Joint 4: (theta4, d4, a4, alpha4)
    (0  , 0      , 0     , 90),       # Joint 5: (theta5, d5, a5, alpha5)
    (0  , 0.08   , 0     , -90)     # Joint 6: (theta6, d6, a6, alpha6)
]
"""
"""
dh_params = [
    (90 , 0.412, -0.1 , -90),    # Joint 1: (theta1, d1, a1, alpha1)
    (-90, 0    , 0.420, 0),     # Joint 2: (theta2, d2, a2, alpha2)
    (0  , 0    , 0    , -90),         # Joint 3: (theta3, d3, a3, alpha3)
    (180, 0.40 , 0    , -90),    # Joint 4: (theta4, d4, a4, alpha4)
    (180, 0    , 0    , -90),       # Joint 5: (theta5, d5, a5, alpha5)
    (180, 0.08 , 0    , 180)     # Joint 6: (theta6, d6, a6, alpha6)
]
"""
# Example joint angles (in degrees)

joint_angles = [-133, -52, -29, -135, -84, 84]  # Example joint angles in degrees

"""
joint_angles = [-105.732, -53, -46, 15, 99, -87.366]
"""
# Compute the end-effector position using forward kinematics
x, y, z, u, v, w = forward_kinematics_epson_vt6(joint_angles, dh_params)

# Print the target (end-effector) position and orientation
print(f"End-effector position: x = {x:.3f} mm, y = {y:.3f} mm, z = {z:.3f} mm")
print(f"End-effector orientation (Euler angles): u = {u:.3f}°, v = {v:.3f}°, w = {w:.3f}°")
