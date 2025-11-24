from imu_calculators import compute_gyroscope_values, compute_2d_mag_values,\
     butter_filter, compute_acceleration_values, quaternion_to_yaw
from simple_gps_calculators import compute_2d_gps_values

import matplotlib.pyplot as plt
import pandas as pd

import numpy as np


#Global variables:
# Driving data
data1 = pd.read_csv('gnss/imu_data/imu_west_seattle_drive.csv')
gps1 = pd.read_csv('gnss/gps_data/gps_west_seattle_drive.csv')
calibration_data1 = pd.read_csv('gnss/imu_data/imu_square_calibration.csv')

# Walking data
data2 = pd.read_csv('gnss/imu_data/imu_lab4_square_walk.csv')
gps2 = pd.read_csv('gnss/gps_data/gps_lab4_square_walk.csv')
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
    plt.title('Magnetic Field: N vs E Components Before Calibration')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)

    # Plot second chart: calibrated mag
    plt.subplot(2,1,2)
    plt.scatter(y_c, x_c, s=2, color='orange')   # E on x-axis, N on y-axis
    plt.xlabel('Magnetic Field East (µT)')
    plt.ylabel('Magnetic Field North (µT)')
    plt.title('Magnetic Field: N vs E Components After Calibration')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_mag_yaw_estimation_before_and_after_calibration_vs_time(dataset, calibration_dataset):
    mag_values = compute_2d_mag_values(dataset,calibration_dataset)

    # Plot first chart: megnetometer heading before correction
    mag_unwrapped = np.unwrap(mag_values['heading_deg'])
    plt.figure(figsize=(8,8))
    plt.subplot(2,1,1)
    plt.plot(mag_values['time'], mag_unwrapped, color='red')  # Time vs heading
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.title('Magnetometer Heading Before Calibration')
    plt.grid(True)
    plt.grid(True)

    # Plot second chart: magnetometer heading after correction
    mag_corr_unwrapped = np.unwrap(mag_values['heading_corr_deg'])
    plt.subplot(2,1,2)
    plt.plot(mag_values['time'], mag_corr_unwrapped, color='green')  # Time vs heading
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.title('Magnetometer Heading After Calibration')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_gyro_yaw_estimation_vs_time(dataset, calibration_dataset):
    gyro_values = compute_gyroscope_values(dataset, calibration_dataset)
    gyro_values_unwrap = np.unwrap(gyro_values['theta_z_deg'])
    # Plot gyro heading vs time
    plt.figure(figsize=(8,6))
    plt.plot(gyro_values['time'], gyro_values_unwrap, color='purple')  # Time vs heading
    plt.ylabel('Gyro Heading (°)')
    plt.xlabel('Time (s)')
    plt.title('Gyroscope Heading vs Time')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_mag_and_gyro_with_complementary_filter2(dataset, calibration_dataset):
    # 1. Get the raw data 
    # (Note: We use the calibration dataset to calculate the bias, but apply it to the dataset)
    mag_vals = compute_2d_mag_values(dataset, calibration_dataset, use_filter=False)    # Calibrated magnetometer values
    gyro_vals = compute_gyroscope_values(dataset, calibration_dataset)  # Calibrated gyroscope values
    imu_yaw = quaternion_to_yaw(dataset) # Get yaw from IMU quaternions

    # Set values (alpha, dt, lpf_cutoff, hpf_cutoff)
    time = mag_vals['time'] # Get time vector
    dt = np.mean(np.diff(time)) # Calculate dt (time difference between samples) for sampling frequency
    lpf_cutoff = 0.4  # Hz
    hpf_cutoff = 0.01  # Hz
    order = 5   # Filter order
    sample_freq = 1/dt  # Sampling frequency in Hz
    alpha = 1  # Complementary filter weight
    

    
    # ---------------------------------------------------------
    # PART A: Low Pass Filter on Magnetometer (for Subplot 1)
    # ---------------------------------------------------------
    # We filter the UNWRAPPED radians to avoid issues at +/- 180 degrees
    mag_heading_unwrap = np.unwrap(mag_vals['heading_corr']) # heading_corr is the calibrated heading in RADIANS
    mag_deg = np.degrees(mag_heading_unwrap)
    # Filter: Cutoff 0.4Hz (Slow/Steady), Sample Rate 1/dt
    _, mag_lpf_rad = butter_filter(mag_heading_unwrap, lpf_cutoff, sample_freq, 'lowpass', order)
    
    # Wrap back to degrees for plotting (-180 to 180)
    mag_lpf_deg = np.degrees((mag_lpf_rad + np.pi) % (2 * np.pi) - np.pi)
    mag_lpf_unwrap = np.unwrap(mag_lpf_deg)

    # ---------------------------------------------------------
    # PART B: High Pass Filter on Gyro (for Subplot 2)
    # ---------------------------------------------------------
    # This is the "Broken" plot that will show the slanted step decay
    gyro_heading_unwrap = np.unwrap(gyro_vals['theta_z'])   # theta_z is the integrated gyro heading in RADIANS
    
    # Filter: Cutoff 0.01Hz (Very slow drift removal)
    _, gyro_hpf_rad = butter_filter(gyro_heading_unwrap, hpf_cutoff, sample_freq, 'highpass', order)
    gyro_hpf_deg = np.degrees(gyro_hpf_rad)

    # ---------------------------------------------------------
    # PART C: Complementary Filter (for Subplot 3)
    # ---------------------------------------------------------
    fused_heading = []
    
    # Start with the Mag heading so we don't start at 0 if we are facing West
    current_heading = mag_vals['heading'][0] 
    
    gyro_rate = gyro_vals['z'] # We need the RATE (deg/s or rad/s), not the angle
    mag_heading = mag_vals['heading']

    for i in range(len(time)):
        # 1. Integrate Gyro (The "New" Motion)
        gyro_step = gyro_rate[i] * dt
        
        # 2. Get Mag reading (The "Anchor")
        mag_step = mag_heading[i]
        
        # 3. Handle Wrapping! (Crucial Math Step)
        # If filter says 10° and Mag says 350°, the difference is +20°, not -340°
        if (mag_step - current_heading) > np.pi:
            mag_step -= 2*np.pi
        elif (mag_step - current_heading) < -np.pi:
            mag_step += 2*np.pi
            
        # 4. The Equation: 98% Gyro Integration + 2% Mag Correction
        current_heading = alpha * (current_heading + gyro_step) + (1 - alpha) * mag_step
        
        fused_heading.append(current_heading)
    
    # Convert to degrees for plotting
    fused_heading_deg = np.degrees((np.array(fused_heading) + np.pi) % (2 * np.pi) - np.pi)
    
    # ---------------------------------------------------------
    # PLOTTING
    # ---------------------------------------------------------
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    
    # Subplot 1: LPF Mag
    axs[0].plot(time, mag_deg, '.', color='lightgray', label='Raw Mag')
    axs[0].plot(time, mag_lpf_unwrap, color='green', linewidth=2, label='LPF Mag')
    axs[0].set_ylabel('Heading (°)')
    axs[0].set_title('Low Pass Filtered Magnetometer')
    axs[0].legend()
    axs[0].grid(True)

    # Subplot 2: HPF Gyro
    axs[1].plot(time, gyro_vals['theta_z_deg'], '--', color='gray', alpha=0.5, label='Integrated Gyro')
    axs[1].plot(time, gyro_hpf_deg, color='purple', label='HPF Gyro')
    axs[1].set_ylabel('Heading (°)')
    axs[1].set_title('High Pass Filtered Gyroscope')
    axs[1].legend()
    axs[1].grid(True)

    # Subplot 3: Complementary Filter
    axs[2].plot(time, fused_heading_deg, color='orange', linewidth=2)
    axs[2].set_ylabel('Heading (°)')
    axs[2].set_title(f'Complementary Filter Output (Fused) α={alpha}')
    axs[2].grid(True)

    # Subplot 4: Comparison
    axs[3].plot(time, imu_yaw['IMU_YAW'],'.', label='IMU yaw', color='lightgray')
    axs[3].plot(time, fused_heading_deg,'--', label='Complementary', alpha=0.8, color='orange')
    axs[3].set_ylabel('Heading (°)')
    axs[3].set_title('Comparison of IMU Yaw and Complementary Filter')
    axs[3].set_xlabel('Time (s)')
    axs[3].legend()
    axs[3].grid(True)

    plt.tight_layout()
    plt.show()

    return np.radians(fused_heading_deg) # Save the fused heading for the Trajectory map

#---------------------------------------------------------------------
def plot_foward_velocity_from_accelerometer(dataset, calibration_dataset):
    
    acc_values = compute_acceleration_values(dataset, calibration_dataset)
    time = acc_values['time']
    # 2. Extract Data for Plotting
    # Forward Velocity (Result of integration)
    velocity_x = acc_values['vel_x']
    true_velocity_x = acc_values['vel_x_true']
    
    # Lateral Acceleration Comparison
    # Observed: What the sensor actually felt (bias corrected)
    # ay_observed = acc_results['acc_y_observed']
    
    # Modeled: What physics says it SHOULD be (v * omega)
    # Note: If your velocity is perfect, this line will match Observed perfectly.
    # ay_modeled = acc_results['acc_y_modeled']

    # -------------------------------------------------------------
    # PLOTTING
    # -------------------------------------------------------------
    fig, axs = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # PLOT 1: Forward Velocity (Fig 4 in your assignment)
    axs[0].plot(time, velocity_x, color='red', linewidth=2, label='Estimated Velocity')
    axs[0].set_ylabel('Velocity (m/s)')
    axs[0].set_title('Estimated Forward Velocity (Integrated from Acceleration)')
    axs[0].grid(True)
    axs[0].legend()

    # PLOT 2: The "Physics Check" (Observed vs Modeled)
    # This proves if your velocity estimate is realistic!
    # axs[1].plot(time, ay_observed, color='gray', alpha=0.7, label='Observed Lateral Accel (Sensor)')
    # axs[1].plot(time, ay_modeled, color='red', linestyle='--', linewidth=1.5, label='Modeled Lateral Accel (v * ω)')
    axs[1].plot(time, true_velocity_x, color='blue', linewidth=2, label='Adjusted Estimated Velocity')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].set_title('Estimated Forward Velocity with corrections (Integrated from True Accel)')
    axs[1].grid(True)
    axs[1].legend()
    # axs[1].set_ylabel('Acceleration (m/s²)')
    # axs[1].set_xlabel('Time (s)')
    # axs[1].set_title('Lateral Acceleration: Observed vs. Modeled\n(Do they match?)')
    # axs[1].grid(True)
    # axs[1].legend()

    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_forward_velocity_from_gps(dataset):
    gps_vals = compute_2d_gps_values(dataset)

    time = gps_vals['time']
    gps_fwd_vel = gps_vals['vel']
    plt.figure(figsize=(10, 6))
    
    plt.plot(time, gps_fwd_vel, color='green', linewidth=1.5, label='GPS Velocity')
    
    plt.title('Forward Velocity Estimated from GPS')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_estimated_trajectory_gps_and_imu(imu_dataset,imu_calibration,gps_data, fused_heading=None):

    gps_vals = compute_2d_gps_values(gps_data)
    acc_vals = compute_acceleration_values(imu_dataset,imu_calibration)

    gps_to_imu_scale = 20.0  # Scale factor to align GPS and IMU distances
    angle_offset_deg = -135.0  # Angle offset in degrees to align headings
    angle_offset_rad = np.radians(angle_offset_deg)

    imu_true_dist = acc_vals['dist_x_true']
    imu_true_head = fused_heading + angle_offset_rad

    N_gps = gps_vals['r_utmn'] 
    E_gps = gps_vals['r_utme']

    # Compute N and E positions using headings and positions
    #We are only interested in forward movement and heading to plot dead-reckoning
    
    dx = np.diff(imu_true_dist)
    dx = np.concatenate(([0], dx))
    
    # Fused
    n_step = dx * np.cos(imu_true_head) /gps_to_imu_scale # Apply scale factor
    e_step = dx * np.sin(imu_true_head) /gps_to_imu_scale
    N_imu = np.cumsum(n_step) 
    E_imu = np.cumsum(e_step)


    # Plot N vs E positions
    plt.figure()
    plt.plot(E_gps,N_gps,'.',label='GPS trajectory')
    plt.plot(E_imu, N_imu, '.', label='IMU trajectory')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Trajectory Plot: N vs E Position, GPS vs IMU')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()



#---------------------------------------------------------------------
def main():
    # Fig. 0: A plot showing the magnetometer data before and after the correction in your report.
    plot_mag_data_before_and_after_correction(calibration_data1, calibration_data1)

    # Fig. 1: The magnetometer yaw estimation before and after hard and soft iron calibration vs. time
    plot_mag_yaw_estimation_before_and_after_calibration_vs_time(data1, calibration_data1)

    # Fig. 2: Plot of gyro yaw estimation vs. time
    plot_gyro_yaw_estimation_vs_time(data1, calibration_data1)
    
    # Fig. 3: Low pass filter of magnetometer data, high pass filter of gyro data,
    # complementary filter output, and IMU heading estimate as 4 subplots on one plot
    # Use a complementary filter to combine the yaw measurements from the magnetometer
    # and yaw rate/gyro to get an improved estimate of the yaw angle 
    # (filter the magnetometer estimate using a low pass filter and gyro estimate using a
    # high pass filter). You might find tools that wrap angle values between -pi and pi useful.
    # Plot the results of the low pass filter, high pass filter & complementary filter together.
    fused_heading = plot_mag_and_gyro_with_complementary_filter2(data1, calibration_data1)
    
    # Fig. 4: Plot of forward velocity from accelerometer before and after any adjustments
    plot_foward_velocity_from_accelerometer(data1, calibration_data1)

    # Fig. 5: Plot of forward velocity from gps
    plot_forward_velocity_from_gps(gps1)

    # Fig. 6: Plot of estimated trajectory from GPS and from IMU velocity/yaw data (2 subplots)
    plot_estimated_trajectory_gps_and_imu(data1,calibration_data1,gps1, fused_heading)


if __name__ == '__main__':
    main()
