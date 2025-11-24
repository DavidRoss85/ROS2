from imu_calculators import compute_gyroscope_values, compute_2d_mag_values,\
     butter_filter, compute_acceleration_values

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

#---------------------------------------------------------------------
def plot_gyro_yaw_estimation_vs_time(dataset, calibration_dataset):
    gyro_values = compute_gyroscope_values(dataset, calibration_dataset)
    # Plot gyro heading vs time
    plt.figure(figsize=(8,6))
    plt.plot(gyro_values['time'], gyro_values['theta_z'], color='purple')  # Time vs heading
    plt.ylabel('Gyro Heading (°)')
    plt.xlabel('Time (s)')
    plt.title('Gyroscope Heading vs Time')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
# def plot_mag_and_gyro_with_complementary_filter(dataset, calibration_dataset):
#     mag_values = compute_2d_mag_values(dataset,calibration_dataset, use_filter=False)
#     gyro_values = compute_gyroscope_values(dataset, calibration_dataset)

#     # Complementary filter parameters
#     order = 5
#     sampl_freq = 40
#     lpf_cutoff = 10
#     hpf_cutoff = .01
#     filter_type = "lowpass"

#     sos, mag_heading_filtered = butter_filter(mag_values['heading_corr_deg'], lpf_cutoff, sampl_freq, filter_type, order)
#     sos, gyro_heading_filtered = butter_filter(gyro_values['theta_corr_z'], hpf_cutoff, sampl_freq, "highpass", order)

#     # Plot all together
#     plt.figure(figsize=(10,10))

#     plt.subplot(4,1,1)
#     plt.plot(mag_values['time'], mag_heading_filtered, color='green')
#     plt.title('Low Pass Filtered Magnetometer Heading')
#     plt.ylabel('Heading (°)')
#     plt.grid(True)

#     plt.subplot(4,1,2)
#     plt.plot(gyro_values['time'], gyro_heading_filtered, color='purple')
#     plt.title('Gyroscope Heading')
#     plt.ylabel('Heading (°)')
#     plt.grid(True)

#     plt.subplot(4,1,3)
#     plt.plot(gyro_values['time'], gyro_values['theta_corr_z'], color='orange')
#     plt.title('Complementary Filter Output Heading')
#     plt.ylabel('Heading (°)')
#     plt.grid(True)

#     # plt.subplot(4,1,4)
#     # imu_heading = comp_heading  # Assuming IMU heading is the complementary filter output
#     # plt.plot(gyro_values['time'], imu_heading, color='blue')
#     # plt.title('IMU Heading Estimate')
#     # plt.ylabel('Heading (°)')
#     # plt.xlabel('Time (s)')
#     # plt.grid(True)

#     plt.tight_layout()
#     plt.show()
def plot_mag_and_gyro_with_complementary_filter(dataset, calibration_dataset):
    # Get raw mag + gyro values (no filtering here)
    mag_values = compute_2d_mag_values(
        dataset, calibration_dataset,
        use_filter=False   # important: don't double-filter
    )
    gyro_values = compute_gyroscope_values(dataset, calibration_dataset)

    # Filter parameters
    order = 5
    sampl_freq = 50     # Hz (approx sample rate of your data)
    cutoff_lpf = 0.1    # Hz for magnetometer low-pass
    cutoff_hpf = 0.3    # Hz for gyro high-pass

    # 1) Low-pass filter magnetometer heading (degrees)
    mag_heading_deg = mag_values['heading_corr_deg']
    _, mag_heading_lpf = butter_filter(
        mag_heading_deg,
        cutoff_lpf, sampl_freq,
        "lowpass", order
    )

    # 2) High-pass filter gyro *integrated* heading (degrees)
    theta_corr_z_deg = np.degrees(gyro_values['theta_corr_z'])
    _, gyro_heading_hpf = butter_filter(
        theta_corr_z_deg,
        cutoff_hpf, sampl_freq,
        "highpass", order
    )

    # Plot for debugging
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(mag_values['time'], mag_heading_deg, label="Mag heading (raw)")
    plt.plot(mag_values['time'], mag_heading_lpf, label="Mag heading (LPF)")
    plt.title('Magnetometer Heading (Raw vs Low-Pass Filtered)')
    plt.ylabel('Heading (°)')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(gyro_values['time'], theta_corr_z_deg, label="Gyro heading (integrated, corr)")
    plt.title('Gyro Heading (Integrated, Bias-Corrected)')
    plt.ylabel('Heading (°)')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(gyro_values['time'], gyro_heading_hpf, label="Gyro heading (HPF)")
    plt.title('High-Pass Filtered Gyro Heading')
    plt.ylabel('Heading (°)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

def main():
    # Fig. 0: A plot showing the magnetometer data before and after the correction in your report.
    # plot_mag_data_before_and_after_correction(calibration_data1, calibration_data1)

    # Fig. 1: The magnetometer yaw estimation before and after hard and soft iron calibration vs. time
    # plot_mag_yaw_estimation_before_and_after_calibration_vs_time(data1, calibration_data1)

    # Fig. 2: Plot of gyro yaw estimation vs. time
    # plot_gyro_yaw_estimation_vs_time(data1, calibration_data1)
    
    # Fig. 3: Low pass filter of magnetometer data, high pass filter of gyro data, complementary filter output, and IMU heading estimate as 4 subplots on one plot
    # Use a complementary filter to combine the yaw measurements from the magnetometer and yaw rate/gyro to get an improved estimate of the yaw angle (filter the magnetometer estimate using a low pass filter and gyro estimate using a high pass filter). You might find tools that wrap angle values between 
    # -pi and pi useful.
    # Plot the results of the low pass filter, high pass filter & complementary filter together.
    plot_mag_and_gyro_with_complementary_filter(data1, calibration_data1)
    
    # Fig. 4: Plot of forward velocity from accelerometer before and after any adjustments
    # Fig. 5: Plot of forward velocity from gps
    # Fig. 6: Plot of estimated trajectory from GPS and from IMU velocity/yaw data (2 subplots)


if __name__ == '__main__':
    main()
