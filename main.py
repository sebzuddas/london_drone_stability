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


    # t_train = np.arange(0, 
    #                     t_end_train, 
    #                     dt)
    
    # x0_train = [-8, 8, 27]
    # t_train_span = (t_train[0], t_train[-1])
    # x_train = solve_ivp(
    #     lorenz, 
    #     t_train_span, 
    #     x0_train, 
    #     t_eval=t_train
    # ).y.T

    # print(data.columns)
    # exit()


    ##############################################################################################################
    # Instantiate and fit the SINDy model
    # feature_library = ps.GeneralizedLibrary([ps.PolynomialLibrary(), ps.FourierLibrary()]))
    model = ps.SINDy(feature_names = ['Roll', 'Pitch', 'Yaw'], 
                     )
    model.fit(data[['Roll', 'Pitch', 'Yaw']].values, t=data['Time'].values)# t can be the dt, so long as it's consistent
    # model.print()
    print(model.coefficients())
    print(model.get_feature_names())

    #np.arange(0, t_end_test, dt)
    sim_test = model.simulate(x0=[0, 0, 0], t=np.arange(0, t_end_test, dt))
    print(sim_test)


def plot_data(data):
    # fig = px.line(data, x='Time', y=['Roll', 'Pitch', 'Yaw'])
    fig = px.line_3d(data, x='Roll', y='Pitch', z='Yaw')
    fig.show()

def loop():
    # The control loop 
    pass

if __name__ == "__main__":

    main()