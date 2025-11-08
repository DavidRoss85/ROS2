import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import cumulative_trapezoid as cumtrapz
import numpy as np

data = pd.read_csv('imu_data/imu_circle_walk.csv')
data = pd.read_csv('imu_data/imu_square_walk.csv')

# Convert timestamps from nanoseconds to seconds
time = data['TIME_NANO'] * 1e-9
time = time - time.iloc[0]

mag_x = data['MAG_X']   # North component
mag_y = data['MAG_Y']   # East component
mag_z = data['MAG_Z']   # Down component

gyro_x = data['ANG_X']  # Angular rate X
gyro_y = data['ANG_Y']  # Angular rate Y
gyro_z = data['ANG_Z']  # Angular rate Z

acc_x = data['LIN_X']  # Linear acceleration X
acc_y = data['LIN_Y']  # Linear acceleration Y
acc_z = data['LIN_Z']  # Linear acceleration Z


#---------------------------------------------------------------------
def plot_magnetic_field_raw():
    # Plot N and E magnetic field components without any calibration:
    plt.figure()
    plt.scatter(mag_y, mag_x, s=2)   # E on x-axis, N on y-axis
    plt.xlabel('Magnetic Field East (µT)')
    plt.ylabel('Magnetic Field North (µT)')
    plt.title('Magnetic Field: N vs E Components')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)
    plt.show()

#---------------------------------------------------------------------
def plot_magnetic_field_calibrated():
    # Plot N and E magnetic field components with calibration:

    #The offset will coreect for hard-iron distortion using mean values    
    offset_x = np.mean(mag_x)
    offset_y = np.mean(mag_y)

    # Subract offsets mean values to shift values to center around zero
    mag_x_corr = mag_x - offset_x
    mag_y_corr = mag_y - offset_y

    # Use the range of corrected values to determine scaling factor
    # We can then re-scale one axis to match the other
    scale_x = (max(mag_x_corr) - min(mag_x_corr)) / 2
    scale_y = (max(mag_y_corr) - min(mag_y_corr)) / 2
    scale_ratio = scale_x / scale_y

    mag_y_corr *= scale_ratio   # adjust one axis so scales match

    # Plot corrected values
    plt.figure()
    plt.scatter(mag_y_corr, mag_x_corr, s=2, color='orange')
    plt.xlabel('Magnetic Field East (µT, corrected)')
    plt.ylabel('Magnetic Field North (µT, corrected)')
    plt.title('Magnetic Field (Calibrated): N vs E Components')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

#---------------------------------------------------------------------
def plot_rotational_rate_x():
    # Plot two charts. 
    # 1: Gyro X rate vs time
    # 2: Integrated angle θₓ vs time

    # Integrate gyro rate to get angle
    theta_x = cumtrapz(gyro_x, time, initial=0)

    # Plot first chart: gyro rate
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, gyro_x)  # Time vs rate
    plt.ylabel('ωₓ (rad/s)')
    plt.title('Gyro X: Rate and Integrated Angle')
    plt.grid(True)

    # Plot second chart: integrated angle
    plt.subplot(2,1,2)
    plt.plot(time, theta_x, color='orange') # Time vs angle
    plt.ylabel('θₓ (rad)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_rotational_rate_y():
    # Plot two charts.
    # 1: Gyro Y rate vs time
    # 2: Integrated angle θᵧ vs time

    # Integrate gyro rate to get angle
    theta_y = cumtrapz(gyro_y, time, initial=0)

    # Plot first chart: gyro rate
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, gyro_y)  # Time vs rate
    plt.ylabel('ωᵧ (rad/s)')
    plt.title('Gyro Y: Rate and Integrated Angle')
    plt.grid(True)

    # Plot second chart: integrated angle
    plt.subplot(2,1,2)
    plt.plot(time, theta_y, color='orange') # Time vs angle
    plt.ylabel('θᵧ (rad)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_rotational_rate_z():
    # Plot three charts.
    # 1: Gyro Z rate vs time
    # 2: Integrated angle θ_z vs time
    # 3: Heading from magnetometer vs time

    # Integrate gyro rate to get angle
    theta_z = cumtrapz(gyro_z, time, initial=0)

    # Compute heading from magnetometer
    heading_mag = np.arctan2(-mag_y, mag_x)
    heading_mag_deg = np.degrees(heading_mag)   # convert to degrees

    # Plot first chart: gyro rate
    plt.figure(figsize=(8,8))
    plt.subplot(3,1,1)
    plt.plot(time, gyro_z)  # Time vs rate
    plt.ylabel('ω_z (rad/s)')
    plt.title('Gyro Z and Magnetometer Heading')
    plt.grid(True)

    # Plot second chart: integrated angle
    plt.subplot(3,1,2)
    plt.plot(time, theta_z, color='orange') # Time vs angle
    plt.ylabel('θ_z (rad)')
    plt.grid(True)

    # Plot third chart: magnetometer heading
    plt.subplot(3,1,3)
    plt.plot(time, heading_mag_deg, color='green')  # Time vs heading
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#---------------------------------------------------------------------
def plot_acceleration_x():
    #Plot 2 charts:
    #1: Acceleration aₓ vs time
    #2: Integrated velocity vₓ vs time

    #
    calibrated_x = acc_x - np.mean(acc_x)  # remove bias
    
    # Integrate acceleration = velocity
    vel_x = cumtrapz(calibrated_x, time, initial=0)

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
def plot_acceleration_y():
    # Plot 2 charts:
    # 1: Acceleration aᵧ vs time
    # 2: Integrated velocity vs time


    calibrated_y = acc_y - np.mean(acc_y)   # remove bias
  
    # Integrate acceleration = velocity
    vel_y = cumtrapz(calibrated_y, time, initial=0)
    
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
def plot_acceleration_z():
    # Plot 2 charts:
    # 1: Acceleration a_z vs time
    # 2: Integrated velocity v_z vs time

    calibrated_z = acc_z - np.mean(acc_z)  # remove bias
    
    # Integrate acceleration = velocity
    vel_z = cumtrapz(calibrated_z, time, initial=0)
    
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
def plot_position_n_vs_e():
    # Plot North vs East position using dead reckoning from accelerometer and gyro/mag headings

    # Remove biases
    cal_x = acc_x - np.mean(acc_x)
    cal_y = acc_y - np.mean(acc_y)

    # Integrate accelration to get velocity
    vel_x = cumtrapz(cal_x, time, initial=0)
    vel_y = cumtrapz(cal_y, time, initial=0)

    # Integrate velocity to get position
    pos_x = cumtrapz(vel_x, time, initial=0)
    pos_y = cumtrapz(vel_y, time, initial=0)

    # Compute headings for mag and gyro
    heading_mag = np.arctan2(-mag_y, mag_x)
    heading_gyro = cumtrapz(gyro_z, time, initial=0)

    # Compute N and E positions using headings and positions
    # This is equivalent to applying a rotation matrix to the (x,y) positions
    # [N E]^T = [cosθ -sinθ; sinθ cosθ] * [x y]^T

    # Magnet
    N_mag = pos_x * np.cos(heading_mag) - pos_y * np.sin(heading_mag)
    E_mag = pos_x * np.sin(heading_mag) + pos_y * np.cos(heading_mag)
    # Gyro
    N_gyro = pos_x * np.cos(heading_gyro) - pos_y * np.sin(heading_gyro)
    E_gyro = pos_x * np.sin(heading_gyro) + pos_y * np.cos(heading_gyro)

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
    # Call all plotting functions
    plot_magnetic_field_raw()
    plot_magnetic_field_calibrated()
    plot_rotational_rate_x()
    plot_rotational_rate_y()
    plot_rotational_rate_z()
    plot_acceleration_x()
    plot_acceleration_y()
    plot_acceleration_z()
    plot_position_n_vs_e()

if __name__ == "__main__":
    main()