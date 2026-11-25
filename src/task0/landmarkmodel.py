import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from  matplotlib.patches import Arc
from matplotlib import transforms
import sympy as sp
arrow = u'$\u2191$'

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

        # plot robot pose
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(x_prime[2])-90)
        plt.scatter(x_prime[0], x_prime[1], marker=rotated_marker, s=80, color='b', alpha=0.3)
    
    # Plot the real pose (green dot) and the landmark (red dot)
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(robot_pose[2])-90)
    plt.scatter(robot_pose[0], robot_pose[1], marker=rotated_marker, s=140, color='r', label="Robot Pose")
    plt.plot(landmark[0], landmark[1], "ro", ms=5, label="Landmark")
    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title(title)
    plt.axis('equal')
    plt.legend(loc='upper left')
    plt.show()
    
def compute_measurement_jacobian():
    """
    Computes and prints the Jacobian matrix H using sympy.
    """
    x, y, theta = sp.symbols("x y theta")
    mx, my = sp.symbols("mx my")
    
    hx = sp.Matrix([
        [sp.sqrt((mx - x) ** 2 + (my - y) ** 2)],
        [sp.atan2(my - y, mx - x) - theta],
    ])
    
    eval_hx = sp.lambdify((x, y, theta, mx, my), hx, "numpy")
    Ht = hx.jacobian(sp.Matrix([x, y, theta]))
    eval_Ht = sp.lambdify((x, y, theta, mx, my), Ht, "numpy")
    print("Jacobian of the measurement model with respect to the state:\n")
    sp.pprint(Ht)
    print("\n\n")
    
    return eval_hx, eval_Ht
def main():
    # Initial state and landmark position
    robot_pose = np.array([3., 4., -math.pi/2])  
    landmark = np.array([5., 2.])

    sigma = np.array([0.4, np.deg2rad(4)]) 
    z = landmark_range_bearing_model(robot_pose, landmark, sigma)
    plot_sampled_poses(robot_pose, z, landmark, sigma, title="Landmark Model Pose Sampling")

    compute_measurement_jacobian()
if __name__ == "__main__":
    main()