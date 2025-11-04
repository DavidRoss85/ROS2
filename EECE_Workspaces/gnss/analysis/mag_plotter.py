import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import cumulative_trapezoid as cumtrapz
import numpy as np

data = pd.read_csv('imu_data/imu_circle_walk.csv')

# Convert timestamps from nanoseconds → seconds
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



def plot_magnetic_field_raw():
    plt.figure()
    plt.scatter(mag_y, mag_x, s=2)   # E on x-axis, N on y-axis
    plt.xlabel('Magnetic Field East (µT)')
    plt.ylabel('Magnetic Field North (µT)')
    plt.title('Magnetic Field: N vs E Components')
    plt.axis('equal')  # keep aspect ratio 1:1
    plt.grid(True)
    plt.show()

def plot_magnetic_field_calibrated():
#Code here for calibration (offsets and scaling factors)
        # --- Hard-iron offset correction (center the data) ---
    offset_x = np.mean(mag_x)
    offset_y = np.mean(mag_y)

    mag_x_corr = mag_x - offset_x
    mag_y_corr = mag_y - offset_y

    # --- Soft-iron scaling correction (make the ellipse round) ---
    scale_x = (max(mag_x_corr) - min(mag_x_corr)) / 2
    scale_y = (max(mag_y_corr) - min(mag_y_corr)) / 2
    scale_ratio = scale_x / scale_y

    mag_y_corr *= scale_ratio   # adjust one axis so scales match

    # --- Plot calibrated data ---
    plt.figure()
    plt.scatter(mag_y_corr, mag_x_corr, s=2, color='orange')
    plt.xlabel('Magnetic Field East (µT, corrected)')
    plt.ylabel('Magnetic Field North (µT, corrected)')
    plt.title('Magnetic Field (Calibrated): N vs E Components')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def plot_rotational_rate_x():
    # Integrate gyro_x over time
    theta_x = cumtrapz(gyro_x, time, initial=0)
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, gyro_x)
    plt.ylabel('ωₓ (rad/s)')
    plt.title('Gyro X: Rate and Integrated Angle')
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time, theta_x, color='orange')
    plt.ylabel('θₓ (rad)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_rotational_rate_y():
    theta_y = cumtrapz(gyro_y, time, initial=0)
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, gyro_y)
    plt.ylabel('ωᵧ (rad/s)')
    plt.title('Gyro Y: Rate and Integrated Angle')
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time, theta_y, color='orange')
    plt.ylabel('θᵧ (rad)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_rotational_rate_z():
    theta_z = cumtrapz(gyro_z, time, initial=0)
    heading_mag = np.arctan2(-mag_y, mag_x)
    heading_mag_deg = np.degrees(heading_mag)

    plt.figure(figsize=(8,8))
    plt.subplot(3,1,1)
    plt.plot(time, gyro_z)
    plt.ylabel('ω_z (rad/s)')
    plt.title('Gyro Z and Magnetometer Heading')
    plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot(time, theta_z, color='orange')
    plt.ylabel('θ_z (rad)')
    plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot(time, heading_mag_deg, color='green')
    plt.ylabel('Mag Heading (°)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_acceleration_x():
    
    calibrated_x = acc_x - np.mean(acc_x)  # remove bias
    vel_x = cumtrapz(calibrated_x, time, initial=0)

    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, calibrated_x)
    plt.ylabel('aₓ (m/s²)')
    plt.title('Acceleration and Velocity – X Axis')
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time, vel_x, color='orange')
    plt.ylabel('vₓ (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_acceleration_y():
    # Remove any DC offset (important before integrating)
    calibrated_y = acc_y - np.mean(acc_y)
    
    # Integrate acceleration → velocity
    vel_y = cumtrapz(calibrated_y, time, initial=0)
    
    # Plot acceleration and velocity
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, calibrated_y)
    plt.ylabel('aᵧ (m/s²)')
    plt.title('Acceleration and Velocity – Y Axis')
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time, vel_y, color='orange')
    plt.ylabel('vᵧ (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_acceleration_z():
    # Remove any DC offset
    calibrated_z = acc_z - np.mean(acc_z)
    
    # Integrate acceleration → velocity
    vel_z = cumtrapz(calibrated_z, time, initial=0)
    
    # Plot acceleration and velocity
    plt.figure(figsize=(8,6))
    plt.subplot(2,1,1)
    plt.plot(time, calibrated_z)
    plt.ylabel('a_z (m/s²)')
    plt.title('Acceleration and Velocity – Z Axis')
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time, vel_z, color='orange')
    plt.ylabel('v_z (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_position_n_vs_e():
    # --- Integrate twice (assuming acc_x, acc_y exist) ---
    cal_x = acc_x - np.mean(acc_x)
    cal_y = acc_y - np.mean(acc_y)
    vel_x = cumtrapz(cal_x, time, initial=0)
    vel_y = cumtrapz(cal_y, time, initial=0)
    pos_x = cumtrapz(vel_x, time, initial=0)
    pos_y = cumtrapz(vel_y, time, initial=0)

    # --- Headings ---
    heading_mag = np.arctan2(-mag_y, mag_x)
    heading_gyro = cumtrapz(gyro_z, time, initial=0)

    # --- Rotate local frame into N/E (if needed) ---
    N_mag = pos_x * np.cos(heading_mag) - pos_y * np.sin(heading_mag)
    E_mag = pos_x * np.sin(heading_mag) + pos_y * np.cos(heading_mag)

    N_gyro = pos_x * np.cos(heading_gyro) - pos_y * np.sin(heading_gyro)
    E_gyro = pos_x * np.sin(heading_gyro) + pos_y * np.cos(heading_gyro)

    # --- Plot ---
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

def plot_position_n_vs_e2():
    # Detrend accelerations
    ax = acc_x - np.mean(acc_x)
    ay = acc_y - np.mean(acc_y)

    # Heading from magnetometer (yaw)
    heading_mag = np.arctan2(-mag_y, mag_x)
    # (Optional) integrate gyro for comparison
    heading_gyro = cumtrapz(gyro_z, time, initial=0)

    # --- Rotate body-frame accel → global-frame accel (mag heading) ---
    aN_mag = ax * np.cos(heading_mag) - ay * np.sin(heading_mag)
    aE_mag = ax * np.sin(heading_mag) + ay * np.cos(heading_mag)

    # --- Integrate twice for position (mag heading) ---
    vN_mag = cumtrapz(aN_mag, time, initial=0)
    vE_mag = cumtrapz(aE_mag, time, initial=0)
    pN_mag = cumtrapz(vN_mag, time, initial=0)
    pE_mag = cumtrapz(vE_mag, time, initial=0)

    # --- Repeat with gyro heading ---
    aN_gyro = ax * np.cos(heading_gyro) - ay * np.sin(heading_gyro)
    aE_gyro = ax * np.sin(heading_gyro) + ay * np.cos(heading_gyro)

    vN_gyro = cumtrapz(aN_gyro, time, initial=0)
    vE_gyro = cumtrapz(aE_gyro, time, initial=0)
    pN_gyro = cumtrapz(vN_gyro, time, initial=0)
    pE_gyro = cumtrapz(vE_gyro, time, initial=0)

    # --- Plot ---
    plt.figure()
    plt.plot(pE_mag, pN_mag, '.', label='Mag Heading')
    plt.plot(pE_gyro, pN_gyro, '.', label='Gyro Heading')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Dead Reckoning: N vs E Position')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
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