import math
import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

def landmark_range_bearing_model(robot_pose, landmark, sigma):
    """
    Samples z from the landmark model for range and bearing.
    """
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]

    r_ = math.dist([x, y], [m_x, m_y]) + np.random.normal(0.0, sigma[0])
    phi_ = math.atan2(m_y - y, m_x - x) - theta + np.random.normal(0.0, sigma[1])
    return np.array([r_, phi_])

def landmark_model_sample_pose(z, landmark, sigma):
    """
    Samples the robot pose from the measurement.
    """
    m_x, m_y = landmark[:]
    sigma_r, sigma_phi = sigma[:]

    gamma_hat = np.random.uniform(0, 2*math.pi)
    r_hat = z[0] + np.random.normal(0, sigma_r)
    phi_hat = z[1] + np.random.normal(0, sigma_phi)

    x_ = m_x + r_hat * math.cos(gamma_hat)
    y_ = m_y + r_hat * math.sin(gamma_hat)
    theta_ = gamma_hat - math.pi - phi_hat

    return np.array([x_, y_, theta_])

def plot_sampled_poses(robot_pose, z, landmark, sigma, title="Landmark Model Pose Sampling"):
    """
    Generates 1000 samples and plots the results.
    """
    # Plot 1000 samples
    for i in range(1000):
        x_prime = landmark_model_sample_pose(z, landmark, sigma)
        plt.scatter(x_prime[0], x_prime[1], marker='.', s=20, color='b', alpha=0.3)
    
    # Plot the real pose (green dot) and the landmark (red dot)
    plt.scatter(robot_pose[0], robot_pose[1], marker='o', s=30, color='g', label="Robot Pose")
    plt.plot(landmark[0], landmark[1], "ro", ms=5, label="Landmark")

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title(title)
    plt.legend(loc='upper right')
    plt.show()

def compute_measurement_jacobian():
    """
    Computes and prints the Jacobian matrix H using sympy.
    """
    x, y, theta = sp.symbols("x y theta")
    mx, my = sp.symbols("m_x m_y")
    
    hx = sp.Matrix([
        [sp.sqrt((mx - x) ** 2 + (my - y) ** 2)],
        [sp.atan2(my - y, mx - x) - theta],
    ])
    
    eval_hx = sp.lambdify((x, y, theta, mx, my), hx, "numpy")

    Ht = hx.jacobian(sp.Matrix([x, y, theta]))
    eval_Ht = sp.lambdify((x, y, theta, mx, my), Ht, "numpy")

    print("\nJacobian Matrix H (Ht):")
    sp.pprint(Ht)
    
    return eval_hx, eval_Ht

def main():
    # Initial state and landmark position
    robot_pose = np.array([3., 4., -math.pi/2])  
    landmark = np.array([5., 2.])

    # Test 1: Uncertainty concentrated on Range
    sigma_range = np.array([0.8, np.deg2rad(2)]) 
    z_range = landmark_range_bearing_model(robot_pose, landmark, sigma_range)
    plot_sampled_poses(robot_pose, z_range, landmark, sigma_range, title="Range Uncertainty")

    # Test 2: Uncertainty concentrated on Bearing (Generates the ring)
    sigma_bearing = np.array([0.05, np.deg2rad(25)]) 
    z_bearing = landmark_range_bearing_model(robot_pose, landmark, sigma_bearing)
    plot_sampled_poses(robot_pose, z_bearing, landmark, sigma_bearing, title="Bearing Uncertainty")

    compute_measurement_jacobian()

if __name__ == "__main__":
    main()