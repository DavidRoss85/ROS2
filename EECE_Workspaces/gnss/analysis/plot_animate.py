# pip install matplotlib pandas

import os
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tf_transformations import euler_from_quaternion
from gps_graph import GPSPlot




CWD = os.getcwd()
OUTDOOR_FILE =  'gps_data/rtk_open.csv'
INDOOR_FILE =  'gps_data/rtk_occluded.csv'
WALKING_FILE = 'gps_data/walking_open_street.csv'
IMU_FILE = 'imu_data/imu_freeform.csv'
# Parameters
WINDOW = 10000          # number of points to show in the moving window
X_COLUMN = "EULER_ROLL"   # column name to plot
Y_COLUMN = "EULER_PITCH"    # column name to plot
Z_COLUMN = "EULER_YAW"      # column name to plot

X_AXIS_FIELD = "TIME_NANO"
Y_MIN = -220
Y_MAX = 220
INTERVAL = 25       # refresh rate in milliseconds
WAIT_AMT = 25e9    # wait time before starting animation in nanoseconds

OUTDOOR_FULL_PATH = os.path.join(CWD,OUTDOOR_FILE)
OCCLUDED_FULL_PATH = os.path.join(CWD,INDOOR_FILE)
WALKING_FULL_PATH = os.path.join(CWD,WALKING_FILE)
IMU_FULL_PATH = os.path.join(CWD,IMU_FILE)

# Path to your CSV file (update this to match your file)
CSV_PATH = IMU_FULL_PATH


# Create the figure
fig, ax = plt.subplots()
# (line1,) = ax.plot([], [], lw=2)
linex, = ax.plot([], [], lw=2, label='Roll',color='red')
liney, = ax.plot([], [], lw=2, label='Pitch',color='green')
linez, = ax.plot([], [], lw=2, label='Yaw',color='blue')

ax.set_xlim(0, WINDOW)
ax.set_ylim(-2, 2)
ax.set_xlabel("Time (ns)")
ax.set_ylabel("Rotational position (degrees)")
ax.set_title("Live Data from IMU - Euler Angles")
ax.set_xticklabels([])
ax.legend(loc='upper right')


start_time = time.time_ns()
wait_time = 0
# Create GPSPlot instance and preprocess data:
data1 = GPSPlot('Animated IMU Data',IMU_FULL_PATH,'blue',x_axis_field='TIME_NANO',y_axis_field='QUAT_X')
data1.convert_quat_to_euler('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W',euler_from_quaternion,degrees=False,new_field_prefix='EULER_')

# Update function
def update(_):
    
    global start_time, wait_time

    if wait_time < WAIT_AMT:
        wait_time += (time.time_ns() - start_time)
        start_time = time.time_ns()
        

    X_MAX = time.time_ns() - start_time
    X_MIN = X_MAX - (WINDOW * 1e6)  # Convert window
    try:

        # Read the CSV file (use only the last few lines for efficiency)
        df = data1.get_data() 


        mask = (df[X_AXIS_FIELD] >= X_MIN) & (df[X_AXIS_FIELD] <= X_MAX)
        df_window = df.loc[mask]

        # xdata = range(len(ydata))
        xaxis_data = df_window[X_AXIS_FIELD].values

        xdata = df_window[X_COLUMN].values
        ydata = df_window[Y_COLUMN].values
        zdata = df_window[Z_COLUMN].values
        

        # Update the plot
        linex.set_data(xaxis_data, xdata)
        liney.set_data(xaxis_data, ydata)
        linez.set_data(xaxis_data, zdata)
        # ax.set_xlim(0, len(ydata))
        ax.set_xlim(X_MIN, X_MAX)
        ax.set_ylim(Y_MIN,Y_MAX) #ax.set_ylim(min(ydata) - 0.1, max(ydata) + 0.1)
    except Exception as e:
        print("Waiting for data...", e)


    return (linex,liney,linez,)

def main():
    start_time = time.time_ns()
    
    ani = FuncAnimation(fig, update, interval=INTERVAL, blit=True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

