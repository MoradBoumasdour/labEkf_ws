import math
from math import cos, sin, sqrt
import numpy as np
import sympy
from sympy import symbols, Matrix
from sensor_msgs.msg import Imu


def sample_velocity_motion_model_5d(x, u, a, dt):
    """Sample velocity motion model.
    Arguments:
    x -- pose of the robot before moving [x, y, theta]
    u -- velocity reading obtained from the robot [v, w]
    sigma -- noise parameters of the motion model [a1, a2, a3, a4, a5, a6] or [std_dev_v, std_dev_w]
    dt -- time interval of prediction
    """

    sigma = np.ones((3))
    if a.shape == u.shape:
        sigma[:-1] = a[:]
        sigma[-1] = a[1] * 0.5
    else:
        sigma[0] = a[0] * u[0] ** 2 + a[1] * u[1] ** 2
        sigma[1] = a[2] * u[0] ** 2 + a[3] * u[1] ** 2
        sigma[2] = a[4] * u[0] ** 2 + a[5] * u[1] ** 2

    # sample noisy velocity commands to consider actuaction errors and unmodeled dynamics
    v_hat = u[0] + np.random.normal(0, sigma[0])
    w_hat = u[1] + np.random.normal(0, sigma[1])
    gamma_hat = np.random.normal(0, sigma[2])

    # compute the new pose of the robot according to the velocity motion model
    r = v_hat / w_hat
    x_prime = x[0] - r * sin(x[2]) + r * sin(x[2] + w_hat * dt)
    y_prime = x[1] + r * cos(x[2]) - r * cos(x[2] + w_hat * dt)
    theta_prime = x[2] + w_hat * dt + gamma_hat * dt

    return np.array([x_prime, y_prime, theta_prime, u[0], u[1]])

def landmark_range_bearing_model(robot_pose, landmark, sigma):
    """""
    Sampling z from landmark model for range and bearing
    """ ""
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]

    r_ = math.dist([x, y], [m_x, m_y]) + np.random.normal(0.0, sigma[0])
    phi_ = math.atan2(m_y - y, m_x - x) - theta + np.random.normal(0.0, sigma[1])
    return np.array([r_, phi_])

def encoder_model(u, sigma):
    """""
    Sampling z from encoder model
    """ ""
    v = u[0]
    w = u[1]

    v_ =v + np.random.normal(0.0, sigma[0])
    w_ = w + np.random.normal(0.0, sigma[1])
    return np.array([v_, w_])

def imu_model(u, sigma):
    """""
    Sampling z from imu model
    """ ""
    w = u[1]

    w_ = w + np.random.normal(0.0, sigma)
    return np.array([w_])

# decorator
def squeeze_sympy_out(func):
    # inner function
    def squeeze_out(*args):
        out = func(*args).squeeze()
        return out

    return squeeze_out


def velocity_mm_simpy_5d():
    x, y, theta, v, w, dt = symbols("x y theta v w dt")
    R = v / w
    beta = theta + w * dt
    gux = Matrix([
        [x - R * sympy.sin(theta) + R * sympy.sin(beta)],
        [y + R * sympy.cos(theta) - R * sympy.cos(beta)],
        [beta],
        [v],
        [w]
    ])
    eval_gux = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), gux, "numpy"))
    
    Gt = gux.jacobian(Matrix([x, y, theta, v, w])) # Matrice 5x5
    eval_Gt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Gt, "numpy"))
    
    Vt = gux.jacobian(Matrix([v, w])) # Matrice 5x2
    eval_Vt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Vt, "numpy"))
    
    return eval_gux, eval_Gt, eval_Vt



def landmark_sm_simpy_5d():
    x, y, theta, v, w, mx, my = symbols("x y theta v w m_x m_y")
    hx = Matrix([
        [sympy.sqrt((mx - x) ** 2 + (my - y) ** 2)],
        [sympy.atan2(my - y, mx - x) - theta],
    ])
    eval_hx = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, mx, my), hx, "numpy"))
    Ht = hx.jacobian(Matrix([x, y, theta, v, w]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, mx, my), Ht, "numpy"))
    return eval_hx, eval_Ht


def odom_sm_simpy_5d():
    """Genera h(x) e il Jacobiano H (2x5) per l'odometria (misura v, w)"""
    x, y, theta, v, w = symbols("x y theta v w")
    
    hx = Matrix([
        [v],
        [w]
    ])
    eval_hx = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w), hx, "numpy"))
    
    Ht = hx.jacobian(Matrix([x, y, theta, v, w])) # Genera in automatico la matrice con gli 1 e 0
    eval_Ht = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w), Ht, "numpy"))
    
    return eval_hx, eval_Ht

def imu_sm_simpy_5d():
    """Genera h(x) e il Jacobiano H (1x5) per l'IMU (misura solo w)"""
    x, y, theta, v, w = symbols("x y theta v w")
    
    hx = Matrix([
        [w]
    ])
    eval_hx = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w), hx, "numpy"))
    
    Ht = hx.jacobian(Matrix([x, y, theta, v, w]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w), Ht, "numpy"))
    
    return eval_hx, eval_Ht