import os
import math
from gps_graph import GPSPlot
from gps_plane import GraphGUI
from tf_transformations import euler_from_quaternion


CWD = os.getcwd()
OUTDOOR_FILE =  'gps_data/rtk_open.csv'
INDOOR_FILE =  'gps_data/rtk_occluded.csv'
WALKING_FILE = 'gps_data/walking_open_street.csv'
IMU_FILE = 'imu_data/imu_circle_walk.csv'

OUTDOOR_FULL_PATH = os.path.join(CWD,OUTDOOR_FILE)
OCCLUDED_FULL_PATH = os.path.join(CWD,INDOOR_FILE)
WALKING_FULL_PATH = os.path.join(CWD,WALKING_FILE)
IMU_FULL_PATH = os.path.join(CWD,IMU_FILE)


######################################################################

def draw_stationary_scatterplot():
    data1 = GPSPlot('OPEN AREA GPS DATA',OUTDOOR_FULL_PATH,'blue')
    data2 = GPSPlot('OCCLUDED AREA GPS DATA', OCCLUDED_FULL_PATH,'red')

    data1.set_name(
        data1.get_name()
        + "\n-Centroid\n - UTME: "
        + str(data1.get_x_mean())
        + "\n - UTMN: "
        + str(data1.get_y_mean())
    )

    data2.set_name(
        data2.get_name()
        + "\n-Centroid\n - UTME: "
        + str(data1.get_x_mean())
        + "\n - UTMN: "
        + str(data2.get_y_mean())
    )
    data1.set_alpha(.15)
    data2.set_alpha(.15)

    chart = GraphGUI('Comparison of GPS Accuracy for Open area vs Occluded area ','Deviation from mean - Easting (m)', 'Deviation from mean - Northing (m)')
    chart.set_x_range(10)
    chart.set_y_range(10)
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.show()

######################################################################

def draw_stationary_altitude_vs_time_plot():
    data1 = GPSPlot('OPEN AREA GPS DATA',OUTDOOR_FULL_PATH,'blue')
    data2 = GPSPlot('OCCLUDED AREA GPS DATA', OCCLUDED_FULL_PATH,'red')

    data1.set_x_axis_field('UTC_SEC')
    data1.set_y_axis_field('ALT')
    data2.set_x_axis_field('UTC_SEC')
    data2.set_y_axis_field('ALT')
    data1.calibrate_graph('line')
    data2.calibrate_graph('line')

    chart = GraphGUI('Comparison of GPS Altitude Data: Open area vs Occluded area','Time (s)', 'Altitude (m)')
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.show('line')

######################################################################

def draw_euclidean_distance_plot_1():
    data1 = GPSPlot('OPEN AREA GPS DATA',OUTDOOR_FULL_PATH,'blue')

    data1.generate_euclidean()

    data1.set_x_axis_field('UTC_SEC')
    data1.set_y_axis_field('EUCLIDEAN')

    data1.calibrate_graph('hist')

    chart = GraphGUI('Euclidean Distance from mean for Open Area', 'Positional Error (m)', 'Rate')
    chart.add_graph(data1)

    chart.show(graph_type='hist',zeroed=False)

######################################################################

def draw_euclidean_distance_plot_2():

    data2 = GPSPlot('OCCLUDED AREA GPS DATA', OCCLUDED_FULL_PATH,'red')

    data2.generate_euclidean()

    data2.set_x_axis_field('UTC_SEC')
    data2.set_y_axis_field('EUCLIDEAN')

    data2.calibrate_graph('hist')

    chart = GraphGUI('Euclidean Distance from mean for Occluded Area', 'Positional Error (m)', 'Rate')
    chart.add_graph(data2)
    chart.show(graph_type='hist',zeroed=False)

######################################################################

def draw_moving_scatterplot():
    data1 = GPSPlot('WALKING GPS DATA',WALKING_FULL_PATH,'green')

    data1.set_name(
        data1.get_name()
        + "\n-Centroid\n - UTME: "
        + str(data1.get_x_mean())
        + "\n - UTMN: "
        + str(data1.get_y_mean())
    )   
    data1.set_x_range(10)
    # data1.set_y_independent(True)
    data1.calculate_LOBF()
    data1.set_lobf_color('red')

    chart = GraphGUI('GPS Accuracy for walking data','Deviation from mean - Easting (m)', 'Deviation from mean - Northing (m)')
    chart.add_graph(data1)
    chart.show()

######################################################################

def draw_moving_altitude_vs_time_plot():
    data1 = GPSPlot('WALKING GPS DATA',WALKING_FULL_PATH,'green')

    data1.set_x_axis_field('UTC_SEC')
    data1.set_y_axis_field('ALT')

    data1.calibrate_graph('line')

    chart = GraphGUI('GPS Altitude Data over time for Walking','Time (s)', 'Altitude (m)')
    chart.add_graph(data1)
    
    chart.show('line')

######################################################################

def draw_gyro_rotational_rate_plot():
    data1 = GPSPlot('x axis rotational rate ',IMU_FULL_PATH,'red',x_axis_field='TIME_NANO',y_axis_field='ANG_X')
    data2 = GPSPlot('y axis rotational rate',IMU_FULL_PATH,'green',x_axis_field='TIME_NANO',y_axis_field='ANG_Y')
    data3 = GPSPlot('z axis rotational rate',IMU_FULL_PATH,'blue',x_axis_field='TIME_NANO',y_axis_field='ANG_Z')

    data1.apply_function_to_data(lambda x: math.degrees(x),'ANG_X','ANG_X_DEGREES')
    data2.apply_function_to_data(lambda y: math.degrees(y),'ANG_Y','ANG_Y_DEGREES')
    data3.apply_function_to_data(lambda z: math.degrees(z),'ANG_Z','ANG_Z_DEGREES')

    data1.set_y_axis_field('ANG_X_DEGREES')
    data2.set_y_axis_field('ANG_Y_DEGREES')
    data3.set_y_axis_field('ANG_Z_DEGREES')

    data1.calibrate_graph('line')
    data3.calibrate_graph('line')
    data2.calibrate_graph('line')

    chart = GraphGUI('Gyro rotational rate','Time (ns)', 'Rotational Rate (deg/s)')
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.add_graph(data3)
    # chart.set_y_range(.01)
    
    chart.show('line',zeroed=False)

######################################################################

def draw_imu_accel_rate_plot():
    data1 = GPSPlot('x axis acceleration rate ',IMU_FULL_PATH,'red',x_axis_field='TIME_NANO',y_axis_field='LIN_X')
    data2 = GPSPlot('y axis acceleration rate',IMU_FULL_PATH,'green',x_axis_field='TIME_NANO',y_axis_field='LIN_Y')
    data3 = GPSPlot('z axis acceleration rate',IMU_FULL_PATH,'blue',x_axis_field='TIME_NANO',y_axis_field='LIN_Z')

    # data1.apply_function_to_data(lambda x: math.degrees(x),'ANG_X','ANG_X_DEGREES')
    # data2.apply_function_to_data(lambda y: math.degrees(y),'ANG_Y','ANG_Y_DEGREES')
    # data3.apply_function_to_data(lambda z: math.degrees(z),'ANG_Z','ANG_Z_DEGREES')

    # data1.set_y_axis_field('ANG_X_DEGREES')
    # data2.set_y_axis_field('ANG_Y_DEGREES')
    # data3.set_y_axis_field('ANG_Z_DEGREES')

    # data1.set_fill_color('red')
    # data2.set_fill_color('green')
    # data3.set_fill_color('lightblue')
    data1.calibrate_graph('line')
    data2.calibrate_graph('line')
    data3.calibrate_graph('line')

    chart = GraphGUI('IMU Accelerometer rate','Time (ns)', 'Acceleration Rate (m/s²)')
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.add_graph(data3)
    # chart.set_y_range(13)
    
    chart.show('line',zeroed=False)

######################################################################

def draw_imu_rotation_angles_plot():
    data1 = GPSPlot('x rotation',IMU_FULL_PATH,'red',x_axis_field='TIME_NANO',y_axis_field='QUAT_X')
    data2 = GPSPlot('y rotation',IMU_FULL_PATH,'green',x_axis_field='TIME_NANO',y_axis_field='QUAT_Y')
    data3 = GPSPlot('z rotation',IMU_FULL_PATH,'blue',x_axis_field='TIME_NANO',y_axis_field='QUAT_Z')

    # Convert quaternions to euler angles, degrees parameter indicates if input is in degrees or radians:
    # This is a long process
    data1.convert_quat_to_euler('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W',euler_from_quaternion,degrees=False,new_field_prefix='EULER_')
    data2.convert_quat_to_euler('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W',euler_from_quaternion,degrees=False,new_field_prefix='EULER_')
    data3.convert_quat_to_euler('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W',euler_from_quaternion,degrees=False,new_field_prefix='EULER_')

    data1.set_y_axis_field('EULER_ROLL')
    data2.set_y_axis_field('EULER_PITCH')  
    data3.set_y_axis_field('EULER_YAW')

    # data1.set_fill_color('red')
    # data2.set_fill_color('green')
    # data3.set_fill_color('lightblue')
    data1.calibrate_graph('line')
    data2.calibrate_graph('line')
    data3.calibrate_graph('line')

    chart = GraphGUI('Rotational postition per axis (x,y,z = roll, pitch, yaw)','Time (ns)', 'Rotation (degrees)')
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.add_graph(data3)
    # chart.set_y_range(10)
    
    chart.show('line',zeroed=True)

######################################################################
def draw_allan_deviation_plot():
    data1 = GPSPlot('Allan Deviation of Gyro X axis',IMU_FULL_PATH,'red',x_axis_field='TIME_NANO',y_axis_field='ANG_X')
    data2 = GPSPlot('Allan Deviation of Gyro Y axis',IMU_FULL_PATH,'green',x_axis_field='TIME_NANO',y_axis_field='ANG_Y')
    data3 = GPSPlot('Allan Deviation of Gyro Z axis',IMU_FULL_PATH,'blue',x_axis_field='TIME_NANO',y_axis_field='ANG_Z')

    data1.generate_allan_dev_series(time_field='TIME_NANO',data_field='ANG_X')
    data2.generate_allan_dev_series(time_field='TIME_NANO',data_field='ANG_Y')
    data3.generate_allan_dev_series(time_field='TIME_NANO',data_field='ANG_Z')
    data1.calibrate_graph('allan')
    data2.calibrate_graph('allan')
    data3.calibrate_graph('allan')


    chart = GraphGUI('Allan Deviation of Gyro axes','Averaging time τ (s)', 'Allan Deviation σ(τ) (deg/s)')
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.add_graph(data3)

    chart.show(graph_type='allan',zeroed=False)
######################################################################
def draw_magnet_field_plot():
    data1 = GPSPlot('x axis magnetic field ',IMU_FULL_PATH,'red',x_axis_field='MAG_X',y_axis_field='MAG_Y')

    data1.calibrate_graph('scatter')
    chart = GraphGUI('Magnetic Field X vs Y','Magnetic Field X (µT)', 'Magnetic Field Y (µT)')
    chart.add_graph(data1)
    chart.show(graph_type='scatter',zeroed=True)

######################################################################
def main():
    # draw_stationary_scatterplot()
    # draw_stationary_altitude_vs_time_plot()
    # draw_euclidean_distance_plot_1()
    # draw_euclidean_distance_plot_2()
    # draw_moving_scatterplot()
    # draw_moving_altitude_vs_time_plot()
    # draw_gyro_rotational_rate_plot()
    # draw_imu_accel_rate_plot()
    # draw_imu_rotation_angles_plot()
    # draw_allan_deviation_plot()
    draw_magnet_field_plot()

######################################################################

if __name__ == '__main__':
    main()