import numpy as np
import math
import pandas as pd
from scipy.integrate import cumulative_trapezoid as cumtrapz
import matplotlib.pyplot as plt

testdata = pd.read_csv('gnss/gps_data/gps_lab4_square_walk.csv')

def compute_2d_gps_values(dataset):
    
    utme= dataset['UTME']
    utmn = dataset['UTMN']
    time = dataset['UTC_SEC'] - dataset['UTC_SEC'][0] # in seconds

    r_utme = utme - utme.mean()
    r_utmn = utmn - utmn.mean()

    dt = np.diff(time)
    dx = np.diff(utme)
    dy = np.diff(utmn)

    vx = dx / dt
    vy = dy / dt
    
    # 4. Calculate Speed (Magnitude of velocity vector)
    # This is your "Forward Velocity" assuming the car is moving forward
    gps_speed = np.sqrt(vx**2 + vy**2)
    
    # 5. Handle Array Sizing
    # np.diff reduces the array size by 1 (100 points -> 99 intervals).
    # To plot it against the original time array, we need to pad it.
    # We add 0 at the start to represent "velocity at time 0 is unknown/0"
    velocity = np.concatenate(([0], gps_speed))
    
    values = dict()
    values['time'] = time
    values['vel'] = velocity
    values['vel_x'] = vx
    values['vel_y'] = vy
    values['utme'] = utme
    values['utmn'] = utmn
    values['r_utme'] = r_utme
    values['r_utmn'] = r_utmn
    return values



def main():
    gps_vals = compute_2d_gps_values(testdata)
    time = gps_vals['time']
    spd = gps_vals['vel']
    plt.plot(time,spd)
    plt.show()

if __name__ == '__main__':
    main()