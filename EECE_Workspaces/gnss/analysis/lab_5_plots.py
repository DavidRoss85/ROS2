from imu_calculators import compute_gyroscope_values, compute_2d_mag_values, compute_acceleration_values

import matplotlib.pyplot as plt
import pandas as pd

import numpy as np


#Global variables:
data1 = pd.read_csv('gnss/imu_data/imu_west_seattle_drive.csv')
calibration_data1 = pd.read_csv('gnss/imu_data/imu_square_calibration.csv')

data2 = pd.read_csv('gnss/imu_data/imu_lab4_square_walk.csv')
calibration_data2 = pd.read_csv('gnss/imu_data/imu_lab4_circle_walk.csv')


#---------------------------------------------------------------------
def plot_mag_data_before_and_after_correction(dataset,calibration_dataset):
    
    mag_values = compute_2d_mag_values(dataset, calibration_dataset)

    x = mag_values['x']
    y = mag_values['y']
    x_c = mag_values['x_corr']
    y_c =  mag_values['y_corr']


    # Plot first chart: uncalibrated mag
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.scatter(y, x, s=2, color='blue')   # E on x-axis, N on y-axis
    plt.xlabel('Magnetic Field East (µT)')
    plt.ylabel('Magnetic Field North (µT)')
    plt.title('Magnetic Field: N vs E Components')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)

    # Plot second chart: calibrated mag
    plt.subplot(2,1,2)
    plt.scatter(y_c, x_c, s=2, color='orange')   # E on x-axis, N on y-axis
    plt.xlabel('Magnetic Field East (µT)')
    plt.ylabel('Magnetic Field North (µT)')
    plt.title('Magnetic Field: N vs E Components')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_mag_yaw_estimation_before_and_after_calibration_vs_time(dataset, calibration_dataset):
    mag_values = compute_2d_mag_values(dataset,calibration_dataset)
    # Plot first chart: megnetometer heading before correction
    plt.figure(figsize=(8,8))
    plt.subplot(2,1,1)
    plt.plot(mag_values['time'], mag_values['heading_deg'], color='red')  # Time vs heading
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.grid(True)

    # Plot second chart: magnetometer heading after correction
    plt.subplot(2,1,2)
    plt.plot(mag_values['time'], mag_values['heading_corr_deg'], color='green')  # Time vs heading
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    # Fig. 0: A plot showing the magnetometer data before and after the correction in your report.
    # plot_mag_data_before_and_after_correction(data1, calibration_data1)

    # Fig. 1: The magnetometer yaw estimation before and after hard and soft iron calibration vs. time
    plot_mag_yaw_estimation_before_and_after_calibration_vs_time(data1, calibration_data1)
    # Fig. 2: Plot of gyro yaw estimation vs. time
    # Fig. 3: Low pass filter of magnetometer data, high pass filter of gyro data, complementary filter output, and IMU heading estimate as 4 subplots on one plot
    # Fig. 4: Plot of forward velocity from accelerometer before and after any adjustments
    # Fig. 5: Plot of forward velocity from gps
    # Fig. 6: Plot of estimated trajectory from GPS and from IMU velocity/yaw data (2 subplots)


if __name__ == '__main__':
    main()
