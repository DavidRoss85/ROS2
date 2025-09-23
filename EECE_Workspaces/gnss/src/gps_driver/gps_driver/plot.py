import os
import matplotlib.pyplot as plt
import pandas as pd

# fig, ax = plt.subplots(figsize=(8, 6))

CWD = os.getcwd()
OUTDOOR_FILE =  'gps_data/stationary_open_2.csv'
INDOOR_FILE =  'gps_data/stationary_indoors_2.csv'
WALKING_FILE = 'gps_data/walking_outdoors.csv'

OUTDOOR_FULL_PATH = os.path.join(CWD,OUTDOOR_FILE)
INDOOR_FULL_PATH = os.path.join(CWD,INDOOR_FILE)
WALKING_FULL_PATH = os.path.join(CWD,WALKING_FILE)


DEFAULT_RANGE = 10

plot_range = DEFAULT_RANGE

##############################################################################################  
##############################################################################################  

class GPSScatterPlot:
    DEFAULT_ALPHA = 0.25

    def __init__(self, name:str='graph', filename:str='', color:str='blue'):
        self.__name = name
        self.__csv_file = filename
        self.__color = color
        self.__x_axis_field = 'UTME'
        self.__y_axis_field = 'UTMN'
        self.__x_axis_zeroed = self.__x_axis_field + '_ZERO'
        self.__y_axis_zeroed = self.__y_axis_field + '_ZERO'
        self.__alpha = self.DEFAULT_ALPHA
        self.__x_mean = 0
        self.__y_mean = 0
        self.__x_min = 0
        self.__x_max = 0
        self.__y_min = 0
        self.__y_max = 0
        self.__data = None
        self.__x_range = 0
        self.__y_range = 0
        self.import_file(self.__csv_file)
        self.calibrate_graph()

##############################################################################################  

    def import_file(self,filename):
        self.__data = pd.read_csv(filename)
##############################################################################################  

    def calibrate_graph(self, option='scatter'):
        self.__x_mean = self.__data[self.__x_axis_field].mean()
        self.__y_mean = self.__data[self.__y_axis_field].mean()
        self.__x_min = self.__data[self.__x_axis_field].min()
        self.__y_min = self.__data[self.__y_axis_field].min()
        self.__x_max = self.__data[self.__x_axis_field].max()
        self.__y_max = self.__data[self.__y_axis_field].max()

        if option.upper() == "SCATTER":
            self.__data[self.__x_axis_zeroed ] = self.__data[self.__x_axis_field] - self.__x_mean
            self.__data[self.__y_axis_zeroed] = self.__data[self.__y_axis_field] - self.__y_mean
            self.__x_range = max(
                abs(self.__x_mean - self.__x_max),
                abs(self.__x_mean - self.__x_min)
            )
            self.__y_range = max(
                abs(self.__y_mean - self.__y_max),
                abs(self.__y_mean - self.__y_min)
            )
        else:
            self.__data[self.__x_axis_zeroed ] = self.__data[self.__x_axis_field] - self.__x_min
            self.__data[self.__y_axis_zeroed] = self.__data[self.__y_axis_field] - self.__y_min
            self.__x_range = abs(self.__x_max - self.__x_min)
            self.__y_range = abs(self.__y_max - self.__y_min)
                
##############################################################################################  
    #Setters:
    def set_name(self, value):
        self.__name = value
    def set_x_axis_field(self, field):
        self.__x_axis_field = field
    def set_y_axis_field(self, field):
        self.__y_axis_field = field
    def set_color(self, value:str):
        self.__color = value
    def set_alpha(self, value):
        self.__alpha = value

##############################################################################################  
    #Getters:
    def get_name(self):
        return self.__name
    def get_csv_file(self):
        return self.__csv_file
    def get_color(self):
        return self.__color
    def get_x_axis_field(self):
        return self.__x_axis_field
    def get_y_axis_field(self):
        return self.__y_axis_field
    def get_x_zeroed_field(self):
        return self.__x_axis_zeroed
    def get_y_zeroed_field(self):
        return self.__y_axis_zeroed
    def get_alpha(self):
        return self.__alpha
    def get_x_mean(self):
        return self.__x_mean
    def get_y_mean(self):
        return self.__y_mean
    def get_x_min(self):
        return self.__x_min
    def get_x_max(self):
        return self.__x_max
    def get_y_min(self):
        return self.__y_min
    def get_y_max(self):
        return self.__y_max
    def get_data(self):
        return self.__data
    def get_x_range(self):
        return self.__x_range
    def get_y_range(self):
        return self.__y_range
    

##############################################################################################  
##############################################################################################  
class ScatterGUI:
    DEFAULT_PLOT_RANGE_SCALE = 1.25

    def __init__(self, name="",x_label="",y_label=""):
        self.__name = name
        self.__plot = plt
        self.__graph_list = dict()
        self.__ax_position = 'zero'
        self.__ay_position = 'zero'
        self.__x_label = x_label
        self.__y_label = y_label
        self.__x_range = DEFAULT_RANGE
        self.__y_range = DEFAULT_RANGE
        self.__x_min = 0
        self.__x_max = 0
        self.__y_min = 0
        self.__y_max = 0
        self.__x_center = 0
        self.__y_center = 0
        # fig, ax = plt.subplots(figsize=(8, 6))
        self.__fig, self.__ax = self.__plot.subplots()

##############################################################################################  
    def add_graph(self,graph:GPSScatterPlot):
        """
        Add a graph to the list of charts to be drawn. Must be done before calling the show routine
        :param graph: A graph object used to draw on the chart
        :return:
        """
        self.__graph_list[graph.get_name()]=graph

##############################################################################################  
    def delete_graph(self,name):
        """
        Remove a graph from the dictionary lis
        :param name: Key reference to the graph
        :return:
        """
        if name in self.__graph_list:
            del self.__graph_list[name]

##############################################################################################  
    def calibrate_center(self, graph:GPSScatterPlot, graph_type="scatter", zeroed=True):
        
        if graph_type.upper() == "SCATTER":
            if zeroed:
                self.__x_range = max(self.__x_range, graph.get_x_range() * self.DEFAULT_PLOT_RANGE_SCALE)
                self.__y_range = max(self.__y_range,graph.get_y_range() * self.DEFAULT_PLOT_RANGE_SCALE)
                self.__x_center = 0
                self.__y_center = 0
            else:
                self.__x_center = graph.get_x_mean()
                self.__y_center = graph.get_y_mean()
        else:
            self.__x_range = max(self.__x_range, graph.get_x_range() * self.DEFAULT_PLOT_RANGE_SCALE)
            self.__y_range = max(self.__y_range, graph.get_y_range() * self.DEFAULT_PLOT_RANGE_SCALE)
            self.__x_min = min(self.__x_min, graph.get_x_min())
            self.__x_max = max(self.__x_max, graph.get_x_max())
            self.__y_min = min(self.__y_min, graph.get_y_min())
            self.__y_max = max(self.__y_max, graph.get_y_max())

 ##############################################################################################     
    def apply_scatterplot_settings(self,zeroed=True):
        #Plot each graph in dictionary:
        for graph in self.__graph_list.values():
            self.__plot.scatter(
                graph.get_data()[graph.get_x_zeroed_field() if zeroed else graph.get_x_axis_field()], 
                graph.get_data()[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()],
                color=graph.get_color(),
                alpha=graph.get_alpha(),
                label=graph.get_name()
            )
            self.calibrate_center(graph,"scatter",zeroed)
        #Set chart properties:
        self.__ax_position = 'center'
        self.__ay_position = 'center'
        self.__plot.xlim(
            self.__x_center - self.__x_range,
            self.__x_center + self.__x_range
        )        
        self.__plot.ylim(
            self.__y_center - self.__y_range,
            self.__y_center + self.__y_range
        )

##############################################################################################  
    def apply_histogram_settings(self,zeroed=True):
        for graph in self.__graph_list.values():
            self.__plot.plot(
                graph.get_data()[graph.get_x_zeroed_field() if zeroed else graph.get_x_axis_field()], 
                graph.get_data()[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()],
                color=graph.get_color(),
                alpha=graph.get_alpha(),
                label=graph.get_name()
            )
            self.__plot.fill_between(
                graph.get_data()[graph.get_x_zeroed_field() if zeroed else graph.get_x_axis_field()], 
                graph.get_data()[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()],
                0,
                color=graph.get_color(),
                alpha=graph.get_alpha(),
                label=graph.get_name()
            )
            self.calibrate_center(graph,"histogram",zeroed)
        #Set chart properties:
        self.__plot.xlim(
            self.__x_min,
            self.__x_min + self.__x_range
        )        
        self.__plot.ylim(
            self.__y_min,
            self.__y_min + self.__y_range
        )

##############################################################################################        
    def show(self,graph_type='scatter', zeroed=True):
        """
        Cycle through all graphs and plot with varying colors
        :return:
        """
        if len(self.__graph_list)==0: return  #Exit if no graphs in dictionary

        if graph_type.upper() == 'SCATTER':
            self.apply_scatterplot_settings(zeroed)
            
        else:
            self.apply_histogram_settings(zeroed)
            

        #Set chart properties:
        self.__ax.spines['left'].set_position(self.__ax_position)
        self.__ax.spines['bottom'].set_position(self.__ay_position)
        self.__ax.spines['top'].set_visible(False)
        self.__ax.spines['right'].set_visible(False)
        # self.__plot.xlim(
        #     self.__x_center - self.__x_range,
        #     self.__x_center + self.__x_range
        # )        
        # self.__plot.ylim(
        #     self.__y_center - self.__y_range,
        #     self.__y_center + self.__y_range
        # )
        self.__plot.legend(loc="upper right")
        self.__plot.title(self.__name)
        self.__plot.ylabel(self.__y_label)
        self.__plot.xlabel(self.__x_label)

        #Display chart
        self.__plot.show()

##############################################################################################  
    #Setters:
    def set_x_axis_position(self, value:str):
        self.__ax_position = value
    def set_y_axis_position(self, value:str):
        self.__ay_position = value

##############################################################################################  
##############################################################################################  
def main():

    data1 = GPSScatterPlot('first_graph',OUTDOOR_FULL_PATH,'blue')
    data2 = GPSScatterPlot('second_graph', INDOOR_FULL_PATH,'red')
    data1.set_x_axis_field('UTC_SEC')
    data1.set_y_axis_field('ALT')
    data2.set_x_axis_field('UTC_SEC')
    data2.set_y_axis_field('ALT')
    data1.calibrate_graph('line')
    data2.calibrate_graph('line')
    chart = ScatterGUI('GPS data','UTM East', 'UTM North')
    chart.add_graph(data1)
    chart.add_graph(data2)
    chart.show('line')

    pass

if __name__ == '__main__':
    main()