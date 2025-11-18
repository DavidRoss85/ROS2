import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import cumulative_trapezoid as cumtrapz
import numpy as np

from gps_graph import GPSPlot
from gps_plane import GraphGUI

#Global variables:
calibration_data2 = pd.read_csv('gnss/imu_data/imu_lab4_circle_walk.csv')
calibration_data1 = pd.read_csv('gnss/imu_data/imu_square_calibration.csv')

data2 = pd.read_csv('gnss/imu_data/imu_lab4_square_walk.csv')
data1 = pd.read_csv('gnss/imu_data/imu_west_seattle_drive.csv')


#---------------------------------------------------------------------
def compute_2d_mag_values(dataset,calibration_data = None):
    """
    Computes 2d magnetometer values including offsets and corrected values
    """
    # Dataset Values
    mag_x = dataset['MAG_X']   # North component
    mag_y = dataset['MAG_Y']   # East component
    mag_z = dataset['MAG_Z']   # Down component

    # Convert timestamps from nanoseconds to seconds
    time = dataset['TIME_NANO'] * 1e-9
    time = time - time.iloc[0]

    # Calibration Values
    if calibration_data is not None:
        cal_x = calibration_data['MAG_X']   # North component
        cal_y = calibration_data['MAG_Y']   # East component
        cal_z = calibration_data['MAG_Z']   # Down component
    else:
        cal_x = mag_x
        cal_y = mag_y
        cal_z = mag_z


    #The offset will correct for hard-iron distortion using mean values    
    offset_x = np.mean(cal_x)
    offset_y = np.mean(cal_y)
    offset_z = np.mean(cal_z)

    # Subract offsets for calibration data
    cal_x_corr = cal_x - offset_x
    cal_y_corr = cal_y - offset_y
    cal_z_corr = cal_z - offset_z

    # Use the range of corrected values to determine scaling factor
    # We can then re-scale one axis to match the other
    scale_x = (max(cal_x_corr) - min(cal_x_corr)) / 2
    scale_y = (max(cal_y_corr) - min(cal_y_corr)) / 2
    scale_ratio = scale_x / scale_y

    # Subract offsets from dataset
    mag_x_corr = mag_x - offset_x
    mag_y_corr = (mag_y - offset_y) * scale_ratio

    values = dict()
    values['x'] = mag_x
    values['y'] = mag_y
    values['z'] = mag_z
    values['x_corr'] = mag_x_corr
    values['y_corr'] = mag_y_corr
    values['offset_x'] = offset_x
    values['offset_y'] = offset_y
    values['scale_x'] = scale_x
    values['scale_y'] = scale_y
    values['scale_ratio'] = scale_ratio
    values['time'] = time
    # Calculate heading:
    values['heading'] = np.arctan2(-mag_y,mag_x)
    values['heading_deg'] = np.degrees(values['heading'])   # convert to degrees
    values['heading_corr'] = np.arctan2(-mag_y_corr,mag_x_corr)
    values['heading_corr_deg'] = np.degrees(values['heading_corr'])   # convert to degrees

    return values

#---------------------------------------------------------------------
def compute_acceleration_values(dataset, calibration_data=None):
    """
    Calculate all acceleration values including integrated velocity and distance
    returns: dictionary
    """
    # Convert timestamps from nanoseconds to seconds
    time = dataset['TIME_NANO'] * 1e-9
    time = time - time.iloc[0]

    # Set dataset values
    acc_x = dataset['LIN_X']  # Linear acceleration X
    acc_y = dataset['LIN_Y']  # Linear acceleration Y
    acc_z = dataset['LIN_Z']  # Linear acceleration Z

    # Set calibration values
    if calibration_data is not None:
        cal_x = calibration_data['LIN_X']
        cal_y = calibration_data['LIN_Y']
        cal_z = calibration_data['LIN_Z']
    else:
        cal_x = acc_x
        cal_y = acc_y
        cal_z = acc_z

    # Calculate bias for each axis
    offset_x = np.mean(cal_x)
    offset_y = np.mean(cal_y)
    offset_z = np.mean(cal_z)

    # Acceleration adjusted for bias:
    acc_corr_x = acc_x - offset_x
    acc_corr_y = acc_y - offset_y
    acc_corr_z = acc_z - offset_z

    # Integrate to find velocity over time
    vel_x = cumtrapz(acc_corr_x,time,initial=0)
    vel_y = cumtrapz(acc_corr_y,time,initial=0)
    vel_z = cumtrapz(acc_corr_z,time,initial=0)

    # Integrate twice to find distance over time
    dist_x = cumtrapz(vel_x, time, initial=0)
    dist_y = cumtrapz(vel_y, time, initial=0)
    dist_z = cumtrapz(vel_z, time, initial=0)

    values = dict()
    values['x'] = acc_x
    values['y'] = acc_y
    values['z'] = acc_z
    values['x_corr'] = acc_corr_x
    values['y_corr'] = acc_corr_y
    values['z_corr'] = acc_corr_z
    values['offset_x'] = offset_x
    values['offset_y'] = offset_y
    values['offset_z'] = offset_z
    values['vel_x'] = vel_x
    values['vel_y'] = vel_y
    values['vel_z'] = vel_z
    values['dist_x'] = dist_x
    values['dist_y'] = dist_y
    values['dist_z'] = dist_z
    values['time'] = time

    return values

#---------------------------------------------------------------------
def compute_gyroscope_values(dataset, calibration_data=None):
    
    # Convert timestamps from nanoseconds to seconds
    time = dataset['TIME_NANO'] * 1e-9
    time = time - time.iloc[0]

    # Set dataset values
    gyro_x = dataset['ANG_X']  # Angular rate X
    gyro_y = dataset['ANG_Y']  # Angular rate Y
    gyro_z = dataset['ANG_Z']  # Angular rate Z

    # Set calibration values
    if calibration_data is not None:
        cal_x = calibration_data['ANG_X']
        cal_y = calibration_data['ANG_Y']
        cal_z = calibration_data['ANG_Z']
    else:
        cal_x = gyro_x
        cal_y = gyro_y
        cal_z = gyro_z

    # Calculate bias for each axis
    offset_x = np.mean(cal_x)
    offset_y = np.mean(cal_y)
    offset_z = np.mean(cal_z)

    gyro_corr_x = gyro_x - offset_x
    gyro_corr_y = gyro_y - offset_y
    gyro_corr_z = gyro_z - offset_z

    # Integrate to get approximate angle:

    # Using non- calibrated values
    theta_x = cumtrapz(gyro_x, time, initial=0)
    theta_y = cumtrapz(gyro_y, time, initial=0)
    theta_z = cumtrapz(gyro_z, time, initial=0)

    #Using calibrated values
    theta_corr_x = cumtrapz(gyro_corr_x, time, initial=0)
    theta_corr_y = cumtrapz(gyro_corr_y, time, initial=0)
    theta_corr_z = cumtrapz(gyro_corr_z, time, initial=0)

    values = dict()
    values['x'] = gyro_x
    values['y'] = gyro_y
    values['z'] = gyro_z
    values['x_corr'] = gyro_corr_x
    values['y_corr'] = gyro_corr_y
    values['z_corr'] = gyro_corr_z
    values['offset_x'] = offset_x
    values['offset_y'] = offset_y
    values['offset_z'] = offset_z
    values['theta_x'] = theta_x
    values['theta_y'] = theta_y
    values['theta_z'] = theta_z
    values['theta_corr_x'] = theta_corr_x
    values['theta_corr_y'] = theta_corr_y
    values['theta_corr_z'] = theta_corr_z
    values['time'] = time

    return values


#---------------------------------------------------------------------
def plot_2d_magnetic_field(dataset, calibration_dataset=None, calibrate = False, plot_color= 'blue'):

    mag_values = compute_2d_mag_values(dataset,calibration_dataset)
    if not calibrate:
        # Plot N and E magnetic field components without any calibration:
        mag_x = mag_values['x']
        mag_y = mag_values['y']
        
    else:
        # Plot values using calibration
        mag_x = mag_values['x_corr']
        mag_y = mag_values['y_corr']
    
    
    plt.figure()
    plt.scatter(mag_y, mag_x, s=2, color=plot_color)   # E on x-axis, N on y-axis
    plt.xlabel('Magnetic Field East (µT)')
    plt.ylabel('Magnetic Field North (µT)')
    plt.title('Magnetic Field: N vs E Components')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)
    plt.show()

#---------------------------------------------------------------------
def plot_rotational_rate_x(dataset, calibration_dataset=None):
    # Plot two charts. 
    # 1: Gyro X rate vs time
    # 2: Integrated angle θₓ vs time

    gyro_values = compute_gyroscope_values(dataset,calibration_dataset)

    # Plot first chart: gyro rate
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(gyro_values['time'], gyro_values['x'])  # Time vs rate
    plt.ylabel('ωₓ (rad/s)')
    plt.title('Gyro X: Rate and Integrated Angle')
    plt.grid(True)

    # Plot second chart: integrated angle
    plt.subplot(2,1,2)
    plt.plot(gyro_values['time'], gyro_values['theta_x'], color='orange') # Time vs angle
    plt.ylabel('θₓ (rad)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_rotational_rate_y(dataset, calibration_dataset=None):
    # Plot two charts.
    # 1: Gyro Y rate vs time
    # 2: Integrated angle θᵧ vs time

    # Calculate gyro values
    gyro_values = compute_gyroscope_values(dataset,calibration_dataset)

    # Plot first chart: gyro rate
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(gyro_values['time'], gyro_values['y'])  # Time vs rate
    plt.ylabel('ωₓ (rad/s)')
    plt.title('Gyro Y: Rate and Integrated Angle')
    plt.grid(True)

    # Plot second chart: integrated angle
    plt.subplot(2,1,2)
    plt.plot(gyro_values['time'], gyro_values['theta_y'], color='orange') # Time vs angle
    plt.ylabel('θₓ (rad)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_rotational_rate_z(dataset, calibration_dataset=None):
    # Plot three charts.
    # 1: Gyro Z rate vs time
    # 2: Integrated angle θ_z vs time
    # 3: Heading from magnetometer vs time

    # Calculate gyro values and Mag values
    gyro_values = compute_gyroscope_values(dataset,calibration_dataset)
    mag_values = compute_2d_mag_values(dataset,calibration_dataset)

    # Plot first chart: gyro rate
    plt.figure(figsize=(8,8))
    plt.subplot(3,1,1)
    plt.plot(gyro_values['time'], gyro_values['z'])  # Time vs rate
    plt.ylabel('ω_z (rad/s)')
    plt.title('Gyro Z and Magnetometer Heading')
    plt.grid(True)

    # Plot second chart: integrated angle
    plt.subplot(3,1,2)
    plt.plot(gyro_values['time'], gyro_values['theta_z'], color='orange') # Time vs angle
    plt.ylabel('θ_z (rad)')
    plt.grid(True)

    # Plot third chart: magnetometer heading
    plt.subplot(3,1,3)
    plt.plot(mag_values['time'], mag_values['heading_corr_deg'], color='green')  # Time vs heading
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_acceleration_x(dataset, calibration_dataset=None):
    #Plot 2 charts:
    #1: Acceleration aₓ vs time
    #2: Integrated velocity vₓ vs time

    accel_values = compute_acceleration_values(dataset, calibration_dataset)
    time = accel_values['time']
    calibrated_x = accel_values['x_corr']
    vel_x = accel_values['vel_x']

    # Plot 1st chart: acceleration
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, calibrated_x)    # Time vs acceleration
    plt.ylabel('aₓ (m/s²)')
    plt.title('Acceleration and Velocity – X Axis')
    plt.grid(True)

    # Plot 2nd chart: velocity
    plt.subplot(2,1,2)
    plt.plot(time, vel_x, color='orange')  # Time vs velocity
    plt.ylabel('vₓ (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_acceleration_y(dataset, calibration_dataset=None):
    # Plot 2 charts:
    # 1: Acceleration aᵧ vs time
    # 2: Integrated velocity vs time


    accel_values = compute_acceleration_values(dataset, calibration_dataset)
    time = accel_values['time']
    calibrated_y = accel_values['y_corr']
    vel_y = accel_values['vel_y']

    # Plot 1st chart: acceleration
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, calibrated_y)    # Time vs acceleration
    plt.ylabel('aᵧ (m/s²)')
    plt.title('Acceleration and Velocity – Y Axis')
    plt.grid(True)

    # Plot 2nd chart: velocity
    plt.subplot(2,1,2)
    plt.plot(time, vel_y, color='orange')   # Time vs velocity
    plt.ylabel('vᵧ (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()
#---------------------------------------------------------------------
def plot_acceleration_z(dataset, calibration_dataset=None):
    # Plot 2 charts:
    # 1: Acceleration a_z vs time
    # 2: Integrated velocity v_z vs time

    accel_values = compute_acceleration_values(dataset, calibration_dataset)
    time = accel_values['time']
    calibrated_z = accel_values['z_corr']
    vel_z = accel_values['vel_z']

    
    # Plot 1st chart: acceleration
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, calibrated_z)    # Time vs acceleration
    plt.ylabel('a_z (m/s²)')
    plt.title('Acceleration and Velocity – Z Axis')
    plt.grid(True)

    # Plot 2nd chart: velocity
    plt.subplot(2,1,2)
    plt.plot(time, vel_z, color='orange')   # Time vs velocity
    plt.ylabel('v_z (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def dead_reckoning_n_vs_e(dataset,calibration_dataset=None):
    # Plot North vs East position using dead reckoning from accelerometer and gyro/mag headings

    mag_values = compute_2d_mag_values(dataset,calibration_dataset)
    acc_values = compute_acceleration_values(dataset, calibration_dataset)
    gyro_values = compute_gyroscope_values(dataset,calibration_dataset)

    pos_x = acc_values['dist_x']
    heading_mag = mag_values['heading_corr']
    heading_gyro = gyro_values['theta_z']
    # Compute N and E positions using headings and positions
    #We are only interested in forward movement and heading to plot dead-reckoning
    
    # Magnet
    dx = np.diff(pos_x)
    dx = np.concatenate(([0], dx))
    n_step = dx * np.sin(heading_mag)
    e_step = dx * np.cos(heading_mag)
    N_mag = np.cumsum(n_step)
    E_mag = np.cumsum(e_step)
    
    # Gyro
    n_step = dx * np.sin(heading_gyro)
    e_step = dx * np.cos(heading_gyro)
    N_gyro = np.cumsum(n_step)
    E_gyro = np.cumsum(e_step)


    # Plot N vs E positions
    plt.figure()
    plt.plot(E_mag, N_mag, '.', label='Mag Heading')
    plt.plot(E_gyro, N_gyro, '.', label='Gyro Heading')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Dead Reckoning: N vs E Position')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()


#---------------------------------------------------------------------
def main():

    plot_2d_magnetic_field(calibration_data1)
    plot_2d_magnetic_field(calibration_data1,calibration_data1,plot_color='orange',calibrate=True)

    plot_rotational_rate_x(calibration_data1)
    plot_rotational_rate_y(calibration_data1)
    plot_rotational_rate_z(calibration_data1)
    plot_acceleration_x(calibration_data1)
    plot_acceleration_y(calibration_data1)
    plot_acceleration_z(calibration_data1)
    dead_reckoning_n_vs_e(calibration_data1)

    # plot_2d_magnetic_field(data1)
    # plot_2d_magnetic_field(data1,calibration_data1,plot_color='orange', calibrate=True)
    # plot_rotational_rate_x(data1,calibration_data1)
    # plot_rotational_rate_y(data1,calibration_data1)
    # plot_rotational_rate_z(data1,calibration_data1)
    # plot_acceleration_x(data1,calibration_data1)
    # plot_acceleration_y(data1,calibration_data1)
    # plot_acceleration_z(data1,calibration_data1)
    dead_reckoning_n_vs_e(data1,calibration_data1)


    # plot_rotational_rate_x(data1)
    # plot_rotational_rate_y(data1)
    # plot_rotational_rate_z(data1)
    # plot_acceleration_x(data1)
    # plot_acceleration_y(data1)
    # plot_acceleration_z(data1)
    dead_reckoning_n_vs_e(data1)

if __name__ == '__main__':
    main()