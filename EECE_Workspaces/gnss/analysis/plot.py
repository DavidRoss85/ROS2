import os
from gps_graph import GPSPlot
from gps_plane import GraphGUI


CWD = os.getcwd()
OUTDOOR_FILE =  'gps_data/open_area_football_field.csv'
INDOOR_FILE =  'gps_data/occluded_area_inside_building.csv'
WALKING_FILE = 'gps_data/walking_open_street.csv'

OUTDOOR_FULL_PATH = os.path.join(CWD,OUTDOOR_FILE)
OCCLUDED_FULL_PATH = os.path.join(CWD,INDOOR_FILE)
WALKING_FULL_PATH = os.path.join(CWD,WALKING_FILE)


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
    data1.set_alpha(.1)
    data2.set_alpha(.1)

    chart = GraphGUI('Comparison of GPS Accuracy for Open area vs Occluded area ','Deviation from mean - Easting (m)', 'Deviation from mean - Northing (m)')
    chart.set_x_range(5)
    chart.set_y_range(5)
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
def main():
    draw_stationary_scatterplot()
    draw_stationary_altitude_vs_time_plot()
    draw_euclidean_distance_plot_1()
    draw_euclidean_distance_plot_2()
    draw_moving_scatterplot()
    draw_moving_altitude_vs_time_plot()

######################################################################

if __name__ == '__main__':
    main()