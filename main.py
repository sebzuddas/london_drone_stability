
import numpy as np
import pandas as pd

import control as ct
import warnings
from contextlib import contextmanager
from copy import copy
from pathlib import Path

import quadprog
import plotly.express as px

from scipy.integrate import solve_ivp
from scipy.linalg import LinAlgWarning
from sklearn.exceptions import ConvergenceWarning
from sklearn.linear_model import Lasso

import pysindy as ps
from pysindy.utils import enzyme
from pysindy.utils import lorenz
from pysindy.utils import lorenz_control

import numpy as np
import control as ct
import control.matlab
import matplotlib.pyplot as plt
from scipy.linalg import kron



def main():

    # SETUP
    # data = (Path() / "../data").resolve()
    # np.random.seed(100)
    # fs = 100 #sample frequency
    dt = 100 #discrete time step in ms
    duration = 180
    t_end_test = dt*duration

    # # Initialize integrator keywords for solve_ivp to replicate the odeint defaults
    # integrator_keywords = {}
    # integrator_keywords["rtol"] = 1e-12
    # integrator_keywords["method"] = "LSODA"
    # integrator_keywords["atol"] = 1e-12

    ########################################################################################

    # simulator = droneSimulator(frequency=fs)

    # data = simulator.generate_data(duration=duration)
    # plot_data(data)

    data = np.load("data/states_inputs_history.npy")
    states = data[:, :3]
    inputs = data[:, 3:]
    
    ##############################################################################################################
    # Instantiate and fit the SINDy model
    feature_library = ps.IdentityLibrary() # ps.GeneralizedLibrary([ps.PolynomialLibrary(), ps.FourierLibrary()])
    sparse_regression_optimizer = ps.STLSQ(threshold=0)
    model = ps.SINDy(feature_names = ['x', 'y', 'z', 'throttle', 'roll', 'pitch'], 
                     feature_library=feature_library, 
                     optimizer = sparse_regression_optimizer
                     )
    
    model.fit(x=states, 
              u=inputs, 
            #   t=data['Time'].values# t can be the dt, so long as it's consistent
              )
    np.save("/Users/sebastianozuddas/Programming/london_drone_stability/data/model_coefficients.npy", model.coefficients())
    

    
    print(inputs.shape[0])  
    
    t=np.arange(0, inputs.shape[0], 1)
    u=np.ones((inputs.shape[0], 3))  # constant input for both input_1 and input_2

    # print(np.shape(t), np.shape(u))
    # exit()

    # print(model.coefficients())
    # sim_test = model.simulate(x0=[0, 0, 0], u=u, t=t)
    
    # plot results 
    # sim_test_df = pd.DataFrame(sim_test)    
    # plot_data(sim_test_df)
        
    coeffs = np.load("/Users/sebastianozuddas/Programming/london_drone_stability/data/model_coefficients.npy")

    
    n = coeffs.shape[0] # number of states nxn
    n_dash = coeffs.shape[1]
    m = u.shape[1] # number of inputs nxp
    
    ############################################# CHECK
    states = model.coefficients()[:, :3]
    inputs = model.coefficients()[:, 3:]

    print('Coefficients: ', coeffs)
    print('States: ', n, n_dash,'\nInputs:', m)
    print(model.coefficients())

    #############################################

    A = states
    B = inputs
    C = np.identity(3) # Output matrix
    p = C.shape[0] # 
    q = 3

    D = np.zeros((p, q))
    
    sysc = ct.matlab.ss(A, B, C, D)
    sysd = ct.matlab.c2d(sysc, dt)
    A, BE, C, D = ct.matlab.ssdata(sysd)


    ############################## TUNING PARAMS
    Q = np.array(
    [   
        [0.01, 0, 0],   # x
        [0, 0.01, 0],   # y
        [0, 0, 0.01]   # z
    ]) #State cost matrix

    R = np.array(
    [
        [0.1, 0, 0], #throttle
        [0, 1, 0], #roll
        [0, 0, 1]  #pitch
    ]) #Input cost matrix
    
    N = 7 # prediction horizon
    
    ############################## 

    # K_inf = ct.dlqr(A, B, Q, R)
    K_2 = -1 * ct.place(A, B,[0.01, 0, 0])

    # print(K_inf)
    # print(K_2)

    A_dlyap = (A+(B@K_2)).T
    Q_dlyap = (Q + K_2.T @ R @ K_2)
    
    try:
        
        P = ct.dlyap(A_dlyap, Q_dlyap)# where P is the solution to the lyapunov equation
        # an extra constraint in python is that Qdlyap has to be symmetric, which isn't true with MATLAB. 
        # In theory it needs to be a Hermitian matrix, a seemingly looser condition that symmetry

        print(P)

    except Exception as e:
        print(f'An exception was raised:\n{e}')
        
        print(f'\nIs Q actually symmetric? {check_symmetric(Q_dlyap)}')

        print(f'\nA_dlyap: \n{A_dlyap}\n Q_dlyap: \n{Q_dlyap}')
        
        exit()

    F, G = predict_mats(A, B, N) # where N is the prediction horizon

    H, L, M_cost = cost_mats(F, G, Q, R, P)

    S = np.linalg.solve(H, -L)

    KN = S[0:m, :]

    spectral_radius(A, B, KN)

    ###### Constraints #########
    
    ### With linear inequalities only

    ############################################################CHECK
    # Pu = np.array([[-1, 0],#throttle
    #                [0, 1],
    #                [-1, 0],#roll
    #                [0, 1],
    #                [-1, 0],#pitch
    #                [0, 1]])
    
    I = np.identity(3)
    Pu = np.vstack((I, -I))

    # qu = np.array([[10000],
    #                [10000],
    #                [10000],
    #                [10000],
    #                [10000],
    #                [10000],])

    qu = np.array([[0],
                   [100],
                   [45],
                   [45],
                   [45],
                   [45],])


    Px = np.vstack((C, -C))

    # qx = np.array([[10000],#x
    #                [10000], 
    #                [10000],#y
    #                [10000],
    #                [10000], #z
    #                 [10000] ])
    
    qx = np.array([[50],#x
                [50], 
                [50],#y
                [50],
                [1], #z
                [25] ])




    Pc, qc, Sc = constraint_mats_2(F, G, Pu=Pu, qu=qu, Px=Px, qx=qx)
    # print(Pc, qc, Sc)


    x0 = np.array([
        [0],#x
        [0],#y
        [0]])#z
    
    u0 = np.array([[0], 
                   [0],
                   [0]]) # system inputs

    timesteps = 150
    k = 0
    x = x0

    xs = []
    us = []
    ys = []

    x_ref = np.array([[10], 
                      [10], 
                      [10]])  # example reference

    if not np.all(Px @ x0 <= qx):
        print("Initial state x0 violates state constraints")

    while k <= timesteps:

        try:

            a = L @ (x - x_ref)            
            PcT = -Pc.T
            b = -(qc + Sc @ x)  # Inequality constraint vector, note the sign change
            a = a.flatten()
            b = b.flatten()

            Uopt = quadprog.solve_qp(H, a, PcT, b) # calculate the optimal control sequence, considering input constraints
            # print(Uopt[0])
            Uopt = Uopt[0]

            uopt = Uopt[:m].reshape(-1, 1)  # Ensure uopt is a column vector
            print(uopt)

            # print(d)
            exit()

            x = A @ x + B @ (uopt) # adding in disturbance, such a model assumes B=E, and this may not be needed. 
            y = C @ x # no Du term

            print(f'Iteration: {k}\n')
            print(f'A: {A}\n')
            print(f'x: {x}\n')
            print(f'B: {B}\n')
            print(f'uopt: {uopt}\n')

            k += 1
            
            xs.append(x)
            us.append(uopt)
            ys.append(y)
        
        except(ValueError) as V:
            print(V)

            print(f'Iteration: {k}\n')
            print(f'A: {A}\n')
            print(f'x: {x}\n')
            print(f'B: {B}\n')
            print(f'uopt: {uopt}\n')

            xs = np.array(xs).squeeze()
            us = np.array(us).squeeze()
            ys = np.array(ys).squeeze()

            plot_data(xs, title="State Trajectory")
            plot_data(us, title="Control Trajectory")
            plot_data(ys, title="Output Trajectory")
            exit()

    print(timesteps)

    xs = np.array(xs)
    us = np.array(us)
    ys = np.array(ys)
    
    
    xs = np.array(xs).squeeze(-1)  # shape: (151, 3)
    us = np.array(us).squeeze(-1)
    ys = np.array(ys).squeeze(-1)

    # results = pd.DataFrame(xs)

    plot_data(xs, title="State Trajectory")
    plot_data(us, title="Control Trajectory")
    plot_data(ys, title="Output Trajectory")


def plot_output(xs:np.array, us:np.array, ys:np.array):
    # # Convert lists to numpy arrays for easier manipulation
    # expect xs, us to be a list of numpy arrays

    # Plotting the results
    plt.figure(figsize=(12, 6))

    # Plotting states
    state_label = ['$\Delta \omega (t)$', '$\Delta p^m (t)$', '$\Delta p^v (t)$', '$\Delta p^{dr} (t)$']
    plt.subplot(3, 1, 1)
    for i in range(xs.shape[1]):
        plt.plot(xs[:, i], label=state_label[i])
    plt.title('State Trajectories')
    plt.xlabel('Time Step')
    plt.ylabel('State Values')
    plt.grid()
    plt.legend()

    # Plotting control inputs
    control_label = ['$\Delta p^{m, ref} (t)$', '$\Delta p^{dr, ref} (t)$']
    plt.subplot(3, 1, 2)
    for i in range(us.shape[1]):
        plt.plot(us[:, i], label=control_label[i])
    plt.title('Control Inputs')
    plt.xlabel('Time Step')
    plt.ylabel('Control Values')
    plt.grid()
    plt.legend()

    # Plotting outputs
    plt.subplot(3, 1, 3)
    plt.plot(ys, label='$\Delta f (t)$')
    plt.title('Outputs')
    plt.xlabel('Time Step')
    plt.ylabel('Output Values')
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()

def plot_data(data, title="3D Line Plot"):
    df = pd.DataFrame(data, columns=['X', 'Y', 'Z'])
    fig = px.line_3d(df, x='X', y='Y', z='Z', title=title)
    fig.show()



def loop():
    # The control loop 
    pass




def spectral_radius(A:np.array, B:np.array, KN:np.array):
     ### Calculating the stability of A+B@KN ###

    Z = A+B@KN
    eigenvals = np.linalg.eigvals(Z)
    sr = np.max(np.abs(eigenvals))
    
    print('Stabilising', sr) if sr < 1 else print('Not Stabilising', sr)

def check_symmetric(a:np.array):
    """
    a: expects a matrix

    check_symmetric checks the matrix for symmetry.
    
    """
    
    return ((a == a.T).all())# check for symmetry for all elements

def predict_mats(A:np.array, B:np.array, N:int):
    """
    Returns the MPC prediction matrices F and G for the system x^+ = A*x + B*u.
    
    Parameters:
    A : 2D array (n x n)
        State matrix.
    B : 2D array (n x m)
        Input matrix.
    N : int
        Prediction horizon length.
    
    Returns:
    F : 2D array
        Prediction matrix F.
    G : 2D array
        Prediction matrix G.
    """
    
    # Dimensions
    n = A.shape[0]
    m = B.shape[1]

    # Allocate matrices
    F = np.zeros((n * N, n))
    G = np.zeros((n * N, m * N))  # Ensure G has enough columns for the full horizon

    # Form row by row
    for i in range(N):
        # F matrix
        F[n * i:n * (i + 1), :] = np.linalg.matrix_power(A, i + 1)
        
        # G matrix
        for j in range(i + 1):
            G[n * i:n * (i + 1), m * j:m * (j + 1)] = np.linalg.matrix_power(A, i - j) @ B

    return F, G

def constraint_mats_2(F, G, Pu, qu, Px=None, qx=None, Pxf=None, qxf=None):
    # Set default values for Px, qx, Pxf, qxf if they are not provided
    if Px is None: Px = np.array([])
    if qx is None: qx = np.array([])
    if Pxf is None: Pxf = np.array([])
    if qxf is None: qxf = np.array([])

    # Input dimension
    m = Pu.shape[1]

    # State dimension
    n = F.shape[1]

    # Horizon length
    N = F.shape[0] // n

    # Number of input constraints
    ncu = len(qu)    

    # Number of state constraints
    ncx = len(qx)

    # Number of terminal constraints
    ncf = len(qxf)

    # Extend state constraints to terminal if terminal ones are not provided
    if ncf == 0 and ncx > 0:
        Pxf = Px
        qxf = qx
        ncf = ncx

    # Input constraints: Build "tilde" (stacked) matrices for constraints over horizon
    Pu_tilde = np.kron(np.eye(N), Pu)
    qu_tilde = np.kron(np.ones((N, 1)), qu)
    Scu = np.zeros((ncu * N, n))

    # State constraints: Build "tilde" (stacked) matrices for constraints over horizon
    
    
    # State constraints
    # Build "tilde" (stacked) matrices for constraints over horizon
    Px0_tilde = np.vstack([Px, np.zeros((ncx * (N - 1) + ncf, n))])
    if ncx > 0:
        Px_tilde = np.hstack([np.kron(np.eye(N - 1), Px), np.zeros((ncx * (N - 1), n))])
    else:
        Px_tilde = np.zeros((ncx, n * N))
    Pxf_tilde = np.hstack([np.zeros((ncf, n * (N - 1))), Pxf])
    Px_tilde = np.vstack([np.zeros((ncx, n * N)), Px_tilde, Pxf_tilde])
    qx_tilde = np.vstack([np.kron(np.ones((N, 1)), qx), qxf])

    print('pu', Pu_tilde.shape, '\npx', Px_tilde.shape,'\nG',G.shape, '\n')

    # Final stack
    if Px_tilde.size == 0:
        Pc = Pu_tilde
        qc = qu_tilde
        Sc = Scu
    else:
        # eliminate x for final form
        Pc = np.vstack([Pu_tilde, Px_tilde @ G])
        qc = np.vstack([qu_tilde, qx_tilde])
        Sc = np.vstack([Scu, -Px0_tilde - Px_tilde @ F])

    return Pc, qc, Sc  

def cost_mats(F, G, Q, R, P):
    """
    Returns the MPC cost matrices H, L, and M for the system.
    
    Parameters:
    F : 2D array
        Prediction matrix F.
    G : 2D array
        Prediction matrix G.
    Q : 2D array
        State weighting matrix.
    R : 2D array
        Input weighting matrix.
    P : 2D array
        Terminal state weighting matrix.
    
    Returns:
    H : 2D array
        Hessian matrix.
    L : 2D array
        Linear term matrix.
    M : 2D array
        Constant term matrix.
    """
    
    from scipy.linalg import block_diag

    # Get dimensions
    n = F.shape[1]
    N = F.shape[0] // n

    # Diagonalize Q and R
    Qd = np.kron(np.eye(N-1), Q) if N > 1 else np.array([])  # Handle case when N=1
    Qd = block_diag(Qd, P) if Qd.size > 0 else P
    Rd = np.kron(np.eye(N), R)

    # Hessian
    H = 2 * (G.T @ Qd @ G) + 2 * Rd

    # Linear term
    L = 2 * (G.T @ Qd @ F)

    # Constant term
    M = F.T @ Qd @ F + Q

    # Make sure the Hessian is symmetric
    H = (H + H.T) / 2

    return H, L, M

def check_ctrb_obsv(A, B, C=None):
    """Function to check controllability and observability for a given ss model"""

    n = A.shape[0]

    # controllability matrix
    Ct = ct.ctrb(A, B)
    rank_Ct = np.linalg.matrix_rank(Ct)
    controllable = (rank_Ct == n)
    
    # observability matrix
    if C is None:
        C = B

    Ob = ct.obsv(A, C)
    rank_Ob = np.linalg.matrix_rank(Ob)
    observable = (rank_Ob == n)


    return controllable, observable




if __name__ == "__main__":

    main()