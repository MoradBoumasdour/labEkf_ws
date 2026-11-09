import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin
import sympy
sympy.init_printing(use_latex='mathjax')
from sympy import symbols, Matrix, latex

def sample_velocity_motion_model( x, u, a, dt):

    v_hat = u[0] + np.random.normal(0, a[0]*u[0]**2 + a[1]*u[1]**2)
    w_hat = u[1] + np.random.normal(0, a[2]*u[0]**2 + a[3]*u[1]**2)
    gamma_hat = np.random.normal(0, a[4]*u[0]**2 + a[5]*u[1]**2)

    r = v_hat/w_hat
    x_prime = x[0] - r*sin(x[2]) + r*sin(x[2]+w_hat*dt)
    y_prime = x[1] + r*cos(x[2]) - r*cos(x[2]+w_hat*dt)
    theta_prime = x[2] + w_hat*dt + gamma_hat*dt

    return np.array([x_prime, y_prime, theta_prime])

def jacocbian_motion_model(x, u, dt):
    x, y, theta, v, w, dt = symbols('x y theta v w dt')
    R = v / w
    beta = theta + w * dt
    gux = Matrix(
        [
            [x - R * sympy.sin(theta) + R * sympy.sin(beta)],
            [y + R * sympy.cos(theta) - R * sympy.cos(beta)],
            [beta],
        ]
    )
    eval_gux = sympy.lambdify((x, y, theta, v, w, dt), gux, 'numpy')
    Gt = gux.jacobian(Matrix([x, y, theta]))
    eval_Gt = sympy.lambdify((x, y, theta, v, w, dt), Gt, "numpy")
    sympy.pprint(Gt)
    Vt = gux.jacobian(Matrix([v, w]))
    eval_Vt = sympy.lambdify((x, y, theta, v, w, dt), Vt, "numpy")

    return eval_gux, eval_Gt, eval_Vt, 


def main():
    plt.close('all')
    n_samples = 500
    dt = 2

    x = [0, 0, 0] # initial pose
    u = [0.8, 0.6] # velocity command (linear and angular)
    a1 = [0.01, 0.01, 0.2, 0.1, 0.0, 0.0] # noise variance
    a2 = [0.2, 0.02, 0.01, 0.01, 0.0, 0.0] 
    x_prime = np.zeros([n_samples, 3])
    for i in range(n_samples):
        x_prime[i,:] = sample_velocity_motion_model(x, u, a1, dt)

    x_second = np.zeros([n_samples, 3])
    for i in range(n_samples):
        x_second[i,:] = sample_velocity_motion_model(x, u, a2, dt)    

    plt.scatter(0, 0, s=20, facecolors='b', edgecolors='b')

    for x_ in x_prime[:200]:
        plt.scatter(x_[0], x_[1],s=10, facecolors='r', edgecolors='none', alpha=0.6)

    for x_ in x_second[:200]:
        plt.scatter(x_[0], x_[1],s=10, facecolors='g', edgecolors='none', alpha=0.6)

    plt.xlim(left=-0.1)
    plt.ylim(bottom=-0.1)
    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling")
    plt.show()

    jacocbian_motion_model(x,u,dt)

if __name__ == "__main__":
    main()