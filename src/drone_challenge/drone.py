import queue
from enum import Enum
from time import sleep

import math
import numpy as np
import generated.drone_pb2 as pb2
import generated.drone_pb2_grpc as pb2_gprc
import grpc

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

class SimulationState(Enum):
    INIT = 0
    STARTED = 1
    ENDED = 3


class DroneClient:
    simulation_state = SimulationState.INIT
    last_update = None

    def __init__(self, host, port, token):
        # instantiate a channel
        self.channel = grpc.insecure_channel("{}:{}".format(host, port))

        # Create a queue for sending messages
        self.send_queue = queue.SimpleQueue()

        # bind the client and the server
        self.stub = pb2_gprc.DroneControllerStub(self.channel)

        # Bind send queue to sending service
        metadata = [("authorization", f"Bearer {token}")]
        self.event_stream = self.stub.DroneConnection(iter(self.send_queue.get, None), metadata=metadata)

        self.history = []
        self.dt = 100 # time steps in ms ##### CHECK ###########################################################################
        self.time_length = 180 #how many seconds should the sim run


    def start(self):

        self.coeffs = np.load("/Users/sebastianozuddas/Programming/london_drone_stability/data/model_coefficients.npy")

        #############################################TODO: CHECK

        dt = 100 #discrete time step in ms

        states = self.coeffs[:, :3]
        inputs = self.coeffs[:, 3:]

        n = self.coeffs.shape[0] # number of states nxn
        n_dash = self.coeffs.shape[1]
        self.m = inputs.shape[1] # number of inputs nxp

        # print('Coefficients: ', self.coeffs)
        # print('States: ', n, n_dash,'\nInputs:', self.m)
        # print(self.coeffs)

        #############################################

        self.A = states
        self.B = inputs
        self.C = np.identity(3) # Output matrix
        p = self.C.shape[0] # 
        q = 3

        self.D = np.zeros((p, q))
        
        sysc = ct.matlab.ss(self.A, self.B, self.C, self.D)
        sysd = ct.matlab.c2d(sysc, dt)#################TODO: check timesteps
        self.A, self.B, self.C, self.D = ct.matlab.ssdata(sysd)


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
        
        N = 7 # prediction horizon #TODO: adjust
        
        ############################## 

        K_2 = -1 * ct.place(self.A, self.B,[0.01, 0, 0])


        A_dlyap = (self.A+(self.B@K_2)).T
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

        F, G = self.predict_mats(self.A, self.B, N) # where N is the prediction horizon

        self.H, self.L, self.M_cost = self.cost_mats(F, G, Q, R, P)

        S = np.linalg.solve(self.H, -self.L)

        KN = S[0:self.m, :]

        self.spectral_radius(self.A, self.B, KN)

        ###### Constraints #########
        
        ### With linear inequalities only

        ############################################################CHECK
        
        I = np.identity(3)
        Pu = np.vstack((I, -I))

        self.qu = np.array([[0],
                    [100],
                    [45],
                    [45],
                    [45],
                    [45],])


        self.Px = np.vstack((self.C, -self.C))
        
        self.qx = np.array([[50],#x
                    [50], 
                    [50],#y
                    [50],
                    [1], #z
                    [25] ])

        self.Pc, self.qc, self.Sc = self.constraint_mats_2(F, G, Pu=Pu, qu=self.qu, Px=self.Px, qx=self.qx)
        # print(Pc, qc, Sc)
        # if not np.all(self.Px @ x0 <= self.qx):
        #     print("Initial state x0 violates state constraints")
        #     exit()

        ######################################################TODO:
        
        # WHERE WE START
        x0 = np.array([
            [0],#x
            [0],#y
            [0]])#z

        # Start the control loop etc
        self.control_loop()

    def receive(self):
        return next(self.event_stream)

    def control_loop(self):
        """Controls the drone until crash, success, or some other failure"""
        iterations = 0
        while iterations <= self.dt * self.time_length:
        # while self.simulation_state != SimulationState.ENDED:
            result = self.receive()

            if self.simulation_state == SimulationState.INIT:
                # Check if the first message is simulate start, and save the goal
                if result.WhichOneof("data") == "start":
                    self.goal_region = result.start.goal ############################################
                    mid_x = (self.goal_region.maximal_point.x + self.goal_region.minimal_point.x)/2
                    mid_y = (self.goal_region.maximal_point.y + self.goal_region.minimal_point.y)/2
                    mid_z = (self.goal_region.maximal_point.z + self.goal_region.minimal_point.z)/2

                    self.x_ref = np.array(
                        [[mid_x], 
                        [mid_y], 
                        [mid_z]])  # example reference

                    # print("Goal:", result.start.goal, "\nBoundary:", result.start.boundary)
                        
                    # print("Goal region: ", self.goal_region)
                
                    # print("Start:", result.start.drone_location.x, result.start.drone_location.y, result.start.drone_location.z)
                    
                    self.position = result.start.drone_location 

                    self.simulation_state = SimulationState.STARTED
                    iterations+=1
                    
                    continue
                else:
                    raise ValueError("Did not receive Simulation Start message as first message")

            # If we receive SimOver ("ended") - end
            if result.WhichOneof("data") == "ended":
                self.simulation_state = SimulationState.ENDED
                print("Simulation ended. Success: ", result.ended.success, " (", result.ended.details, ")")
                print(dc.history)
                numpy_history = np.array(self.history)
                np.save("/Users/sebastianozuddas/Programming/london_drone_stability/data/states_inputs_history.npy", numpy_history)
                return

            ############################ Update - pass into PID loop ######################################
            if result.WhichOneof("data") == "update":
                self.position = result.update.drone_location

                # print(f"Position: ", self.position.x, self.position.y, self.position.z)
                self.x = np.array(
                        [[self.position.x], 
                        [self.position.y], 
                        [self.position.z]])
                # Run PID loop
                self.control_drone(iterations)
                iterations+=1


    def mpc_loop(self):
        

        a = self.L @ (self.x_ref - self.x)#TODO: check        
        PcT = -self.Pc.T
        b = -(self.qc + self.Sc @ self.x)  # Inequality constraint vector, note the sign change
        a = a.flatten()
        b = b.flatten()

        Uopt = quadprog.solve_qp(self.H, a, PcT, b) # calculate the optimal control sequence, considering input constraints
        # print(Uopt[0])
        Uopt = Uopt[0]
        Uopt = Uopt.astype(np.int32)

        uopt = Uopt[:self.m].reshape(-1, 1)  # Ensure uopt is a column vector
        # self.x = self.A @ self.x + self.B @ (uopt)
        
        # y = self.C @ x 

        return uopt

    def control_drone(self, iterations):
        # print(f"Start of control_drone. Position: ", self.position.x, self.position.y, self.position.z)

        # throttle = int(50*math.sin(0.5*iterations)+50) # if self.position.z < self.goal_region.maximal_point.z else 0
        # pitch = int(math.cos(0.5*iterations)*45)
        # roll = int(math.sin(0.5*iterations)*45)

        # print("Requesting control input of:")
        # print(f"{throttle=}")
        # print(f"{pitch=}")
        # print(f"{roll=}")
        

        uopt = self.mpc_loop()
        
        throttle=uopt[0][0]
        pitch=uopt[1][0]
        roll=uopt[2][0]
        #print(throttle, pitch, roll)
        print("Input:",uopt, "\nStates:",self.x, "\nReference:",self.x_ref, "\nDifference:", self.x_ref - self.x)

        control = pb2.DroneClientMsg(
            throttle=throttle,
            pitch=pitch,
            roll=roll,
        )
        self.send_queue.put(control)

        self.history.append((self.position.x, self.position.y, self.position.z ,throttle, roll, pitch))


    def spectral_radius(self, A:np.array, B:np.array, KN:np.array):
        ### Calculating the stability of A+B@KN ###

        Z = A+B@KN
        eigenvals = np.linalg.eigvals(Z)
        sr = np.max(np.abs(eigenvals))
        
        print('Stabilising', sr) if sr < 1 else print('Not Stabilising', sr)

    def check_symmetric(self, a:np.array):
        """
        a: expects a matrix

        check_symmetric checks the matrix for symmetry.
        
        """
        
        return ((a == a.T).all())# check for symmetry for all elements

    def predict_mats(self, A:np.array, B:np.array, N:int):
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

    def constraint_mats_2(self, F, G, Pu, qu, Px=None, qx=None, Pxf=None, qxf=None):
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

    def cost_mats(self, F, G, Q, R, P):
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

    def check_ctrb_obsv(self, A, B, C=None):
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
    host = "172.237.101.153"
    port = 10301
    token = "MPC_Man"

    dc = DroneClient(host, port, token)
    dc.start()
    
