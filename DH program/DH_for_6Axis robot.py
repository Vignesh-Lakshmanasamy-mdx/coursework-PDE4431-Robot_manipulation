import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics_6dof_rrr(theta1, theta2, theta3, theta4, theta5, theta6, l1, l2, l3, l4, l5, l6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6):
    """
    Computes the forward kinematics for a 6-DOF RRRRRR robot with twists.
    Returns the end-effector position (x, y, z).
    """
    # Initialize position (fixed base at origin)
    x, y, z = 0, 0, l1

    # Link 2 end
    x1 = x + l2 * np.cos(theta1) * np.cos(theta2 + alpha2)
    y1 = y + l2 * np.sin(theta1) * np.cos(theta2 + alpha2)
    z1 = z + l2 * np.sin(theta2 + alpha2)

    # Link 3 end
    x2 = x1 + l3 * np.cos(theta1) * np.cos(theta2 + theta3 + alpha3)
    y2 = y1 + l3 * np.sin(theta1) * np.cos(theta2 + theta3 + alpha3)
    z2 = z1 + l3 * np.sin(theta2 + theta3 + alpha3)

    # Link 4 end
    x3 = x2 + l4 * np.cos(theta1) * np.cos(theta2 + theta3 + theta4 + alpha4)
    y3 = y2 + l4 * np.sin(theta1) * np.cos(theta2 + theta3 + theta4 + alpha4)
    z3 = z2 + l4 * np.sin(theta2 + theta3 + theta4 + alpha4)

    # Link 5 end
    x4 = x3 + l5 * np.cos(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + alpha5)
    y4 = y3 + l5 * np.sin(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + alpha5)
    z4 = z3 + l5 * np.sin(theta2 + theta3 + theta4 + theta5 + alpha5)

    # Link 6 end (End-effector)
    x5 = x4 + l6 * np.cos(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + theta6 + alpha6)
    y5 = y4 + l6 * np.sin(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + theta6 + alpha6)
    z5 = z4 + l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6 + alpha6)

    return x5, y5, z5


def plot_robot(theta1, theta2, theta3, theta4, theta5, theta6, l1, l2, l3, l4, l5, l6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, x_target, y_target, z_target):
    """
    Plots the robot configuration given joint angles, link lengths, and twists.
    """
    # Base position
    x0, y0, z0 = 0, 0, 0

    # Link 1 end
    x1, y1, z1 = 0, 0, l1

    # Link 2 end
    x2 = x1 + l2 * np.cos(theta1) * np.cos(theta2 + alpha2)
    y2 = y1 + l2 * np.sin(theta1) * np.cos(theta2 + alpha2)
    z2 = z1 + l2 * np.sin(theta2 + alpha2)

    # Link 3 end
    x3 = x2 + l3 * np.cos(theta1) * np.cos(theta2 + theta3 + alpha3)
    y3 = y2 + l3 * np.sin(theta1) * np.cos(theta2 + theta3 + alpha3)
    z3 = z2 + l3 * np.sin(theta2 + theta3 + alpha3)

    # Link 4 end
    x4 = x3 + l4 * np.cos(theta1) * np.cos(theta2 + theta3 + theta4 + alpha4)
    y4 = y3 + l4 * np.sin(theta1) * np.cos(theta2 + theta3 + theta4 + alpha4)
    z4 = z3 + l4 * np.sin(theta2 + theta3 + theta4 + alpha4)

    # Link 5 end
    x5 = x4 + l5 * np.cos(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + alpha5)
    y5 = y4 + l5 * np.sin(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + alpha5)
    z5 = z4 + l5 * np.sin(theta2 + theta3 + theta4 + theta5 + alpha5)

    # Link 6 end (End-effector)
    x6 = x5 + l6 * np.cos(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + theta6 + alpha6)
    y6 = y5 + l6 * np.sin(theta1) * np.cos(theta2 + theta3 + theta4 + theta5 + theta6 + alpha6)
    z6 = z5 + l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6 + alpha6)

    # Plot the robot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot links
    ax.plot([x0, x1], [y0, y1], [z0, z1], 'ro-', label="Link 1 (Base to Joint 1)")
    ax.plot([x1, x2], [y1, y2], [z1, z2], 'go-', label="Link 2 (Joint 1 to Joint 2)")
    ax.plot([x2, x3], [y2, y3], [z2, z3], 'bo-', label="Link 3 (Joint 2 to Joint 3)")
    ax.plot([x3, x4], [y3, y4], [z3, z4], 'yo-', label="Link 4 (Joint 3 to Joint 4)")
    ax.plot([x4, x5], [y4, y5], [z4, z5], 'co-', label="Link 5 (Joint 4 to Joint 5)")
    ax.plot([x5, x6], [y5, y6], [z5, z6], 'mo-', label="Link 6 (Joint 5 to End-effector)")

    # Plot target and end-effector
    ax.scatter([x_target], [y_target], [z_target], color="orange", s=50, label="Target")
    ax.scatter([x6], [y6], [z6], color="purple", s=50, label="End-effector")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.legend()
    ax.set_title("6-DOF RRRRRR Robot Configuration with Twists")
    plt.show()


# Example usage
if __name__ == "__main__":
    # Link lengths
    l1, l2, l3, l4, l5, l6 = 0.257, 0.247, 0.426, 0.082, 0.304, 0.067

    # Joint angles (radians)
    theta1, theta2, theta3, theta4, theta5, theta6 = np.radians([-105, 37, -46, 15, 99, -87])

    # Joint twists
    alpha1 = np.radians(-90)
    alpha2 = np.radians(0)
    alpha3 = np.radians(0)
    alpha4 = np.radians(0)
    alpha5 = np.radians(0)
    alpha6 = np.radians(0)

    # Calculate the target position using forward kinematics
    x_target, y_target, z_target = forward_kinematics_6dof_rrr(theta1, theta2, theta3, theta4, theta5, theta6, l1, l2, l3, l4, l5, l6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6)

    print("Target position (End-effector position):")
    print(f"x: {x_target:.2f}, y: {y_target:.2f}, z: {z_target:.2f}")

    # Plot the robot configuration
    plot_robot(theta1, theta2, theta3, theta4, theta5, theta6, l1, l2, l3, l4, l5, l6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, x_target, y_target, z_target)
