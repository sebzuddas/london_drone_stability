from droneSimulator import droneSimulator
import numpy as np
import pandas as pd


import warnings
from contextlib import contextmanager
from copy import copy
from pathlib import Path


import plotly.express as px

from scipy.integrate import solve_ivp
from scipy.linalg import LinAlgWarning
from sklearn.exceptions import ConvergenceWarning
from sklearn.linear_model import Lasso

import pysindy as ps
from pysindy.utils import enzyme
from pysindy.utils import lorenz
from pysindy.utils import lorenz_control


def main():

    # SETUP
    data = (Path() / "../data").resolve()
    np.random.seed(100)
    fs = 100 #sample frequency
    dt = 1/fs #discrete time step
    duration = 1
    t_end_test = fs*duration

    # # Initialize integrator keywords for solve_ivp to replicate the odeint defaults
    # integrator_keywords = {}
    # integrator_keywords["rtol"] = 1e-12
    # integrator_keywords["method"] = "LSODA"
    # integrator_keywords["atol"] = 1e-12

    ########################################################################################

    simulator = droneSimulator(frequency=fs)

    data = simulator.generate_data(duration=duration)

    ##############################################################################################################
    # Instantiate and fit the SINDy model
    feature_library = ps.IdentityLibrary() # ps.GeneralizedLibrary([ps.PolynomialLibrary(), ps.FourierLibrary()])
    sparse_regression_optimizer = ps.STLSQ(threshold=0)
    model = ps.SINDy(feature_names = ['Roll', 'Pitch', 'Yaw'], 
                     feature_library=feature_library, 
                     optimizer = sparse_regression_optimizer
                     )
    
    model.fit(x=data[['Roll', 'Pitch', 'Yaw']].values, 
              u=data[['input_1', 'input_2']].values, 
              t=data['Time'].values# t can be the dt, so long as it's consistent
              )

    t=np.arange(0, t_end_test, dt)
    u=np.ones((len(t), 2))  # constant input for both input_1 and input_2

    # print(model.coefficients())
    sim_test = model.simulate(x0=[0, 0, 0], u=u, t=t)
    
    # plot results 
    # sim_test_df = pd.DataFrame(sim_test, columns=['Roll', 'Pitch', 'Yaw'])    
    # plot_data(sim_test_df)
    
    
    
    coeffs = model.coefficients()
    
    n = coeffs.shape[0] # number of states nxn
    n_dash = coeffs.shape[1]
    m = u.shape[1] # number of inputs nxp

    print('Coefficients: ', coeffs)
    print('States: ', n, n_dash,'\nInputs:', m)
    
    exit()
    
    # model.print()
    # print(model.coefficients())
    # print(model.inputs())
    
    # A: coefficients for Roll, Pitch, Yaw
    A_cols = [i for i, f in enumerate(features) if f in ['Roll', 'Pitch', 'Yaw']]
    B_cols = [i for i, f in enumerate(features) if f in ['input_1', 'input_2']]

    A = coeffs[:, A_cols]
    B = coeffs[:, B_cols]

    print(A)
    print(B)

    exit()

    #np.arange(0, t_end_test, dt)
    # sim_test = model.simulate(x0=[0, 0, 0], t=np.arange(0, t_end_test, dt))
    # print(sim_test)


def plot_data(data):
    # fig = px.line(data, x='Time', y=['Roll', 'Pitch', 'Yaw'])
    fig = px.line_3d(data, x='Roll', y='Pitch', z='Yaw')
    fig.show()

def loop():
    # The control loop 
    pass

if __name__ == "__main__":

    main()