import numpy as np
import pandas as pd


class droneSimulator:
    def __init__(self, frequency:int,):

        # self.dt = dt
        # self.time = time
        self.frequency = frequency

    def generate_data(self, duration, noise_level = 0.01):
        
        time = np.linspace(0, duration, int(duration*self.frequency), False)
        
        roll = np.sin(2*np.pi*time)  # generates roll data with some noise
        pitch = np.cos(2*np.pi*time)  # generates pitch data with some noise
        yaw = (roll + pitch)  # yaw is a function of roll and pitch in this case

        # Adding white noise to the roll, pitch and yaw
        roll_noise  = np.random.normal(0, noise_level, len(time))
        pitch_noise = np.random.normal(0, noise_level, len(time))
        yaw_noise   = np.random.normal(0, noise_level, len(time))
        
        roll   += roll_noise
        pitch  += pitch_noise
        yaw    += yaw_noise

        df = pd.DataFrame({'Time': time, 'Roll': roll, 'Pitch': pitch, 'Yaw': yaw})
        return df
