import numpy as np
import matplotlib.pyplot as plt


def dh_transform(a, alpha, d, theta):
    """
    Compute the Denavit-Hartenberg transformation matrix.
    a: link length
    alpha: link twist
    d: link offset
    theta: joint angle
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics_6dof_epson(joint_angles, dh_params):
    """
    Computes the forward kinematics for a 6-DOF Epson VT6 robot.
    joint_angles: List of joint angles (theta1, theta2, ..., theta6)
    dh_params: List of tuples (a, alpha, d, theta) for each joint
    Returns: end-effector position and orientation.
    """
    transformation_matrix = np.eye(4)  # Start with identity matrix

    for i in range(6):
        a, alpha, d, theta = dh_params[i]
        transformation_matrix = np.dot(transformation_matrix, dh_transform(a, alpha, d, joint_angles[i] + theta))

    # Extract position from the final transformation matrix
    position = transformation_matrix[:3, 3]
    orientation = transformation_matrix[:3, :3]

    return position, orientation


def inverse_kinematics_6dof_epson(x_target, y_target, z_target, orientation_target, dh_params):
    """
    Solves inverse kinematics for the Epson VT6 robot.
    Uses numerical methods (e.g., Newton-Raphson or optimization) to find joint angles.
    """
    from scipy.optimize import fmin

    # Define the cost function to minimize
    def cost_function(joint_angles):
        position, orientation = forward_kinematics_6dof_epson(joint_angles, dh_params)
        position_error = np.linalg.norm(position - np.array([x_target, y_target, z_target]))
        orientation_error = np.linalg.norm(orientation_target - orientation)
        return position_error + orientation_error

    # Initial guess for joint angles (zero angles)
    initial_guess = np.zeros(6)

    # Minimize the cost function to find joint angles
    result = fmin(cost_function, initial_guess)

    return result


def plot_robot_6dof(joint_angles, dh_params):
    """
    Plots the 6-DOF Epson VT6 robot configuration in 3D space.
    joint_angles: List of joint angles
    dh_params: List of DH parameters
    """
    # Compute the forward kinematics
    position, _ = forward_kinematics_6dof_epson(joint_angles, dh_params)

    # Plot the robot in 3D space
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot each link
    x, y, z = [0], [0], [0]  # Start from the base

    for i in range(6):
        a, alpha, d, theta = dh_params[i]
        transform_matrix = dh_transform(a, alpha, d, joint_angles[i] + theta)
        position_i = np.dot(transform_matrix, np.array([0, 0, 0, 1]))
        x.append(position_i[0])
        y.append(position_i[1])
        z.append(position_i[2])

    ax.plot(x, y, z, 'bo-', label="Robot Links")
    ax.scatter([position[0]], [position[1]], [position[2]], color="red", s=100, label="End-Effector")

    # Plot target position
    ax.scatter([x_target], [y_target], [z_target], color="orange", s=100, label="Target Position")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_title("6-DOF Epson VT6 Robot Configuration")

    plt.legend()
    plt.show()


# Example usage for Epson VT6 Robot
if __name__ == "__main__":
    # DH parameters: (a, alpha, d, theta) for each joint
    dh_params = [
        (0, -np.pi/2, 0.5, 0),    # Joint 1
        (0.5, 0, 0, np.pi/2),     # Joint 2
        (0.5, 0, 0, 0),           # Joint 3
        (0, np.pi/2, 0, 0),       # Joint 4
        (0, -np.pi/2, 0.5, 0),    # Joint 5
        (0, 0, 0, 0)              # Joint 6
    ]

    # Target position and orientation for the end-effector
    x_target, y_target, z_target = 630, -320, 1050
    orientation_target = np.eye(3)  # Identity matrix for orientation (no rotation)

    try:
        # Solve for joint angles using inverse kinematics
        joint_angles = inverse_kinematics_6dof_epson(x_target, y_target, z_target, orientation_target, dh_params)
        print(f"Joint Angles: {joint_angles}")

        # Validate by computing the forward kinematics
        position, orientation = forward_kinematics_6dof_epson(joint_angles, dh_params)
        print(f"End-Effector Position: {position}")
        print(f"End-Effector Orientation: \n{orientation}")

        # Plot the robot's configuration
        plot_robot_6dof(joint_angles, dh_params)

    except Exception as e:
        print(f"Error: {e}")
