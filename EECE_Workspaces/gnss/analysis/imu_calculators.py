import numpy as np
from scipy.integrate import cumulative_trapezoid as cumtrapz
from scipy.signal import butter, sosfilt

#---------------------------------------------------------------------
#Modified from DelftStack https://www.delftstack.com/howto/python/low-pass-filter-python/
#Original author: Vaibhhav Khetarpal
def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
    sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

#---------------------------------------------------------------------
def compute_2d_mag_values(dataset,calibration_data = None,use_filter = False, filter_order = 5, cutoff_freq = 3.667, sampl_freq = 200, filter_type = "lowpass"):
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

    #Filter Values:
    if use_filter:
        sos, mag_x = butter_filter(mag_x, cutoff_freq, sampl_freq, filter_type, filter_order)
        sos, mag_y = butter_filter(mag_y, cutoff_freq, sampl_freq, filter_type, filter_order)
        sos, mag_z = butter_filter(mag_z, cutoff_freq, sampl_freq, filter_type, filter_order)

    # Calibration Values
    cal_x, cal_y, cal_z = None, None, None
    if calibration_data is not None:
        if use_filter:
            _, cal_x = butter_filter(calibration_data['MAG_X'], cutoff_freq, sampl_freq, filter_type, filter_order)
            _, cal_y = butter_filter(calibration_data['MAG_Y'], cutoff_freq, sampl_freq, filter_type, filter_order)
            _, cal_z = butter_filter(calibration_data['MAG_Z'], cutoff_freq, sampl_freq, filter_type, filter_order)
        else:
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
    cal_x, cal_y, cal_z = None, None, None
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
def compute_gyroscope_values(dataset,calibration_data = None,use_filter = False, filter_order = 5, cutoff_freq = 3.667, sampl_freq = 200, filter_type = "highpass"):
    
    # Convert timestamps from nanoseconds to seconds
    time = dataset['TIME_NANO'] * 1e-9
    time = time - time.iloc[0]

    # Set dataset values
    gyro_x = dataset['ANG_X']  # Angular rate X
    gyro_y = dataset['ANG_Y']  # Angular rate Y
    gyro_z = dataset['ANG_Z']  # Angular rate Z

    #Filter Values:
    if use_filter:
        sos, gyro_x = butter_filter(gyro_x, cutoff_freq, sampl_freq, filter_type, filter_order)
        sos, gyro_y = butter_filter(gyro_y, cutoff_freq, sampl_freq, filter_type, filter_order)
        sos, gyro_z = butter_filter(gyro_z, cutoff_freq, sampl_freq, filter_type, filter_order)

    # Calibration Values
    cal_x, cal_y, cal_z = None, None, None
    if calibration_data is not None:
        if use_filter:
            _, cal_x = butter_filter(calibration_data['ANG_X'], cutoff_freq, sampl_freq, filter_type, filter_order)
            _, cal_y = butter_filter(calibration_data['ANG_Y'], cutoff_freq, sampl_freq, filter_type, filter_order)
            _, cal_z = butter_filter(calibration_data['ANG_Z'], cutoff_freq, sampl_freq, filter_type, filter_order)
        else:
            cal_x = calibration_data['ANG_X']   # North component
            cal_y = calibration_data['ANG_Y']   # East component
            cal_z = calibration_data['ANG_Z']   # Down component
    else:
        cal_x = gyro_x
        cal_y = gyro_y
        cal_z = gyro_z

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
    values['theta_z_deg'] = np.degrees(theta_z)
    values['theta_corr_x'] = theta_corr_x
    values['theta_corr_y'] = theta_corr_y
    values['theta_corr_z'] = theta_corr_z
    values['time'] = time

    return values