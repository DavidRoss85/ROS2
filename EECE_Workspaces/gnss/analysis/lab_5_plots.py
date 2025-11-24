from imu_calculators import compute_gyroscope_values, compute_2d_mag_values,\
     butter_filter, compute_acceleration_values, quaternion_to_yaw

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
def plot_mag_and_gyro_with_complementary_filter(dataset, calibration_dataset):

    mag_values = compute_2d_mag_values(
        dataset,
        calibration_dataset, 
        use_filter=True,
        filter_order=5,
        cutoff_freq=.3,
        sampl_freq=40,
        filter_type="lowpass"
    )
    gyro_values = compute_gyroscope_values(
        dataset, 
        calibration_dataset,
        use_filter=True,
        filter_order=5,
        cutoff_freq=.5,
        sampl_freq=40,
        filter_type="highpass"
    )

    unfiltered_gyro_values = compute_gyroscope_values(
        dataset, 
        calibration_dataset,
        use_filter=False,
        filter_order=5,
        cutoff_freq=.5,
        sampl_freq=40,
        filter_type="highpass"
    )
    # sos, mag_heading_filtered = butter_filter(mag_values['heading_corr_deg'], lpf_cutoff, sampl_freq, filter_type, order)
    sos, gyro_heading_filtered = butter_filter(unfiltered_gyro_values['theta_z'], .5, 40, "highpass", 5)

    # Plot all together
    plt.figure(figsize=(10,10))

    plt.subplot(4,1,1)
    plt.plot(mag_values['time'], mag_values['heading_corr_deg'], color='green')
    plt.title('Low Pass Filtered Magnetometer Heading')
    plt.ylabel('Heading (°)')
    plt.grid(True)

    plt.subplot(4,1,2)
    plt.plot(gyro_values['time'], gyro_values['theta_z'], color='purple')
    plt.title('Gyroscope Heading')
    plt.ylabel('Heading (°)')
    plt.grid(True)

    plt.subplot(4,1,3)
    plt.plot(gyro_values['time'], gyro_heading_filtered, color='orange')
    plt.title('Complementary Filter Output Heading')
    plt.ylabel('Heading (°)')
    plt.grid(True)

    # plt.subplot(4,1,4)
    # imu_heading = comp_heading  # Assuming IMU heading is the complementary filter output
    # plt.plot(gyro_values['time'], imu_heading, color='blue')
    # plt.title('IMU Heading Estimate')
    # plt.ylabel('Heading (°)')
    # plt.xlabel('Time (s)')
    # plt.grid(True)

#-----------------------------
def plot_mag_and_gyro_with_complementary_filter2(dataset, calibration_dataset):
    # 1. Get the raw data 
    # (Note: We use the calibration dataset to calculate the bias, but apply it to the dataset)
    mag_vals = compute_2d_mag_values(dataset, calibration_dataset, use_filter=False)
    gyro_vals = compute_gyroscope_values(dataset, calibration_dataset)
    
    time = mag_vals['time']
    # Calculate dt (time difference between samples)
    dt = np.mean(np.diff(time)) 
    
    # ---------------------------------------------------------
    # PART A: Low Pass Filter on Magnetometer (for Subplot 1)
    # ---------------------------------------------------------
    # We filter the UNWRAPPED radians to avoid issues at +/- 180 degrees
    mag_heading_unwrap = np.unwrap(mag_vals['heading_corr'])
    
    # Filter: Cutoff 0.5Hz (Slow/Steady), Sample Rate 1/dt
    _, mag_lpf_rad = butter_filter(mag_heading_unwrap, .4, 1/dt, 'lowpass', 5)
    
    # Wrap back to degrees for plotting (-180 to 180)
    mag_lpf_deg = np.degrees((mag_lpf_rad + np.pi) % (2 * np.pi) - np.pi)

    # ---------------------------------------------------------
    # PART B: High Pass Filter on Gyro (for Subplot 2)
    # ---------------------------------------------------------
    # This is the "Broken" plot that will show the slanted step decay
    gyro_heading_unwrap = np.unwrap(gyro_vals['theta_z'])
    
    # Filter: Cutoff 0.01Hz (Very slow drift removal)
    _, gyro_hpf_rad = butter_filter(gyro_heading_unwrap, 0.01, 1/dt, 'highpass', 5)
    gyro_hpf_deg = np.degrees(gyro_hpf_rad)

    # ---------------------------------------------------------
    # PART C: Complementary Filter (for Subplot 3)
    # ---------------------------------------------------------
    alpha = 1
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
    
    # Get yaw from IMU
    imu_yaw = quaternion_to_yaw(dataset)
    # ---------------------------------------------------------
    # PLOTTING
    # ---------------------------------------------------------
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    
    # Subplot 1: LPF Mag
    axs[0].plot(time, mag_vals['heading_corr_deg'], '.', color='lightgray', label='Raw Mag')
    axs[0].plot(time, mag_lpf_deg, color='green', linewidth=2, label='LPF Mag')
    axs[0].set_ylabel('Heading (°)')
    axs[0].set_title('Low Pass Filtered Magnetometer (Removes Jitter)')
    axs[0].legend()
    axs[0].grid(True)

    # Subplot 2: HPF Gyro
    axs[1].plot(time, gyro_vals['theta_z_deg'], '--', color='gray', alpha=0.5, label='Integrated Gyro')
    axs[1].plot(time, gyro_hpf_deg, color='purple', label='HPF Gyro')
    axs[1].set_ylabel('Heading (°)')
    axs[1].set_title('High Pass Filtered Gyroscope (Shows Decay/Slant)')
    axs[1].legend()
    axs[1].grid(True)

    # Subplot 3: Complementary Filter
    axs[2].plot(time, fused_heading_deg, color='orange', linewidth=2)
    axs[2].set_ylabel('Heading (°)')
    axs[2].set_title('Complementary Filter Output (Fused)')
    axs[2].grid(True)

    # Subplot 4: Comparison
    axs[3].plot(time, fused_heading_deg, label='Complementary', color='orange')
    axs[3].plot(time, imu_yaw['IMU_YAW'], label='IMU yaw', alpha=0.3)
    axs[3].set_ylabel('Heading (°)')
    axs[3].set_title('Comparison')
    axs[3].set_xlabel('Time (s)')
    axs[3].legend()
    axs[3].grid(True)

    plt.tight_layout()
    plt.show()

    return fused_heading_deg # Save this! We need it for the Trajectory map later.

def main():
    # Fig. 0: A plot showing the magnetometer data before and after the correction in your report.
    plot_mag_data_before_and_after_correction(calibration_data1, calibration_data1)

    # Fig. 1: The magnetometer yaw estimation before and after hard and soft iron calibration vs. time
    plot_mag_yaw_estimation_before_and_after_calibration_vs_time(data1, calibration_data1)

    # Fig. 2: Plot of gyro yaw estimation vs. time
    plot_gyro_yaw_estimation_vs_time(data1, calibration_data1)
    
    # Fig. 3: Low pass filter of magnetometer data, high pass filter of gyro data, complementary filter output, and IMU heading estimate as 4 subplots on one plot
    # Use a complementary filter to combine the yaw measurements from the magnetometer and yaw rate/gyro to get an improved estimate of the yaw angle (filter the magnetometer estimate using a low pass filter and gyro estimate using a high pass filter). You might find tools that wrap angle values between 
    # -pi and pi useful.
    # Plot the results of the low pass filter, high pass filter & complementary filter together.
    # plot_mag_and_gyro_with_complementary_filter(data2, calibration_data2)
    plot_mag_and_gyro_with_complementary_filter2(data2, calibration_data2)
    
    # Fig. 4: Plot of forward velocity from accelerometer before and after any adjustments
    # Fig. 5: Plot of forward velocity from gps
    # Fig. 6: Plot of estimated trajectory from GPS and from IMU velocity/yaw data (2 subplots)


if __name__ == '__main__':
    main()
