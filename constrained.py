""" A version of main.py but with no libary, pure linear algebra"""

import numpy as np
import control as ct
import control.matlab
import matplotlib.pyplot as plt
from scipy.linalg import kron
import quadprog
import math
import random


def main():

    Ts = 0.1 # sampling time

    #### Defining the Model #####

    # Model params
    M = 8
    D = 0.8 
    R = 0.05 
    T_t = 0.5 
    T_g = 0.2 
    T_dr = 0.25 # demand response time (s), standard is 0.25


    #### Continuous Time System Models ####

    Ac = np.array(
        [[-D/M, 1/M, 0, 1/M],
        [0, -1/T_t, 1/T_t, 0], 
        [-1/(R*T_g), 0, -1/T_g, 0], 
        [0, 0, 0, -1/T_dr]]) #System Matrix

    Bc = np.array(
        [[0, 0],
        [0, 0],
        [1/T_g, 0],
        [0, 1/T_dr]]) # Input Matrix

    Ec = np.array(
        [[-1/M],
        [0],
        [0],
        [0]]) # Disturbances
    
    Cc = np.array([[50], [0], [0], [0]]) # Output matrix
    Cc = Cc.T

    # print(Cc)
    # exit()

    # D = np.zeros(p, n)
    
    n = Ac.shape[0] # number of states nxn
    m = Bc.shape[1] # number of inputs nxp
    p = Cc.shape[0] # 
    
    Dc = np.zeros((p, m+1))
    # print(Dc)

    BcEc = np.concatenate((Bc, Ec), axis=1)
    # print(BcEc)
    
    ##### Discrete-time model generation #####

    #obtains the discrete time prediction model
    sysc = ct.matlab.ss(Ac, BcEc, Cc, Dc)
    sysd = ct.matlab.c2d(sysc, Ts)
    A, BE, C, D = ct.matlab.ssdata(sysd)
    
    # Separate inputs B and disturbances E
    # print(BE)
    B = BE[:, 0:m]
    
    #TODO: implement E
    E = BE[:, m:]

    # print(A)
    # print(B)
    # print(C)
    # print(E)

    ### Weighting matrices ###
    # these are some tuning parameters
    
    Q = np.array(
    [   
        [0.0001, 0, 0, 0],   # \Delta \omega (t)
        [0, 0.0001, 0, 0],   # \Delta p^m
        [0, 0, 0.00001, 0],   # \Delta p^v
        [0, 0, 0, 0.0000001]    # \Delta p^dr
    ]) #State cost matrix

    R = np.array(
    [
        [0.5, 0], #\Delta p^m, ref - financially cheaper to do
        [0, 2]  #\Delta p^dr, ref - financially more expensive
    ]) #Input cost matrix
    
    N = 18 # prediction horizon
    
    #K_inf = ct.dlqr(A, B, Q, R)
    K_2 = -1 * ct.place(A, B,[0.01, 0.01, 0.1, 0.01])

    # print(K_inf)
    # print(K_2)

    A_dlyap = (A+(B@K_2)).T
    Q_dlyap = (Q + K_2.T @ R @ K_2)
    
    try:
        
        P = ct.dlyap(A_dlyap, Q_dlyap)# where P is the solution to the lyapunov equation
        # an extra constraint in python is that Qdlyap has to be symmetric, which isn't true with MATLAB. 
        # In theory it needs to be a Hermitian matrix, a seemingly looser condition that symmetry

    except Exception as e:
        print(f'An exception was raised:\n{e}')
        
        print(f'\nIs Q actually symmetric? {check_symmetric(Q_dlyap)}')

        print(f'\nA_dlyap: \n{A_dlyap}\n Q_dlyap: \n{Q_dlyap}')
        
        exit()

    # print(P)

    F, G = predict_mats(A, B, N) # where N is the prediction horizon

    H, L, M_cost = cost_mats(F, G, Q, R, P)

    # print(F, G, H, L, M_cost)

    S = np.linalg.solve(H, -L)
    # S becomes an m*N matrix (ie it is the set of optimal control inputs along the horizon, calculated in a receding manner)
    # print(S)
    
    KN = S[0:m, :]
    
    # check for stability:
    spectral_radius(A, B, KN)

    ###### Constraints #########
    
    ### With linear inequalities only

    Pu = np.array([[1, 0],#+I
                   [0, 1],
                   [-1, 0],#-I
                   [0, -1]])

    qu = np.array([[0.5],
                   [1],
                   [0.5],
                   [0]])
    
    Px = np.vstack((C, -C))
    
    qx = np.array([[4],
                   [0.5]])

    Pc, qc, Sc = constraint_mats_2(F, G, Pu=Pu, qu=qu, Px=Px, qx=qx)

    # system starting states, dw(t), dpm(t), dpv(t), dpdr(t)
    x0 = np.array([
        [0.01],
        [1],
        [1],
        [-1]])
    
    u0 = np.array([[0], 
                   [0]]) # system inputs, dpm,ref(t) dpdr,ref(t)

    timesteps = 150
    k = 0
    x = x0

    xs = []
    us = []
    ys = []

    while k <= timesteps:

        try:

            a = L @ x
            PcT = -Pc.T
            b = -(qc + Sc @ x)  # Inequality constraint vector, note the sign change
            a = a.flatten()
            b = b.flatten()

            Uopt = quadprog.solve_qp(H, a, PcT, b) # calculate the optimal control sequence, considering input constraints
            # print(type(Uopt[0]))
            Uopt = Uopt[0]

            uopt = Uopt[:m].reshape(-1, 1)  # Ensure uopt is a column vector

            d = np.array([[abs(random.gauss(mu=0.1, sigma=0.1))], 
                          [abs(random.gauss(mu=0.1, sigma=0.1))]])
            
            # print(d)
            # exit()

            x = A @ x + B @ (uopt + d) # adding in disturbance, such a model assumes B=E, and this may not be needed. 
            y = C @ x # no Du term

            # print(f'Iteration: {k}\n')
            # print(f'A: {A}\n')
            # print(f'x: {x}\n')
            # print(f'B: {B}\n')
            # print(f'uopt: {uopt}\n')

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

            plot_output(xs, us, ys, costed_control_inputs_cumsum)
            exit()

    #####TODO: Implement evaluating the solution via a summation of the dr curve. 

    xs = np.array(xs).squeeze()
    us = np.array(us).squeeze()
    ys = np.array(ys).squeeze()

    prices = np.array([0.12, 3.37]) # demand response = 3.37, paying power producers = 0.12
    costed_control_inputs = np.multiply(np.abs(us), prices)# make all values positive
    costed_control_inputs_cumsum = np.cumsum(np.abs(costed_control_inputs), axis=0)# get the cumulative summation to see how costs grow
    total_cost_vector = np.sum(costed_control_inputs_cumsum, axis=1)    
    combined_costs = np.column_stack((costed_control_inputs_cumsum, total_cost_vector))
    
    # costed_control_inputs_cumsum = np.concatenate(costed_control_inputs_cumsum, total_cost_vector)
    # print(costed_control_inputs_cumsum)
    # exit()
    total_cost_of_inputs = costed_control_inputs_cumsum[-1]# get last element from cumsum
    total_cost_of_inputs_summed = np.sum(total_cost_of_inputs)# get total cost of controller
    print(total_cost_of_inputs_summed)

    plot_output(xs, us, ys, combined_costs)

def spectral_radius(A:np.array, B:np.array, KN:np.array):
     ### Calculating the stability of A+B@KN ###

    Z = A+B@KN
    eigenvals = np.linalg.eigvals(Z)
    sr = np.max(np.abs(eigenvals))
    
    print('Stabilising', sr) if sr < 1 else print('Not Stabilising', sr)

def plot_output(xs:np.array, us:np.array, ys:np.array, costed_control_inputs_cumsum:np.array):
    # # Convert lists to numpy arrays for easier manipulation
    # expect xs, us to be a list of numpy arrays

    # Plotting the results
    plt.figure(figsize=(12, 6))

    # Plotting states
    state_label = ['$\Delta \omega (t)$', '$\Delta p^m (t)$', '$\Delta p^v (t)$', '$\Delta p^{dr} (t)$']
    plt.subplot(4, 1, 1)
    for i in range(xs.shape[1]):
        plt.plot(xs[:, i], label=state_label[i])
    plt.title('State Trajectories')
    plt.xlabel('Time Step')
    plt.ylabel('State Values')
    plt.grid()
    plt.legend()

    # Plotting control inputs
    control_label = ['$\Delta p^{m, ref} (t)$', '$\Delta p^{dr, ref} (t)$']
    plt.subplot(4, 1, 2)
    for i in range(us.shape[1]):
        plt.plot(us[:, i], label=control_label[i])
    plt.title('Control Inputs')
    plt.xlabel('Time Step')
    plt.ylabel('Control Values')
    plt.grid()
    plt.legend()
    


    # Plotting outputs
    price_label = ['m,ref cost (£)', 'm, dr cost (£)', 'Total Cost (£)']
    plt.subplot(4, 1, 3)
    for i in range(len(price_label)):
        plt.plot(costed_control_inputs_cumsum[:, i], label=price_label[i])
    plt.title('Control Action Cost')
    plt.xlabel('Time Step')
    plt.ylabel('Total Cost')
    plt.grid()
    plt.legend()

    # Plotting outputs
    plt.subplot(4, 1, 4)
    plt.plot(ys, label='$\Delta f (t)$')
    plt.title('Outputs')
    plt.xlabel('Time Step')
    plt.ylabel('Output Values')
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()

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

if __name__ == '__main__':
    main()