import matplotlib.pyplot as plt
from gps_graph import GPSPlot

DEFAULT_RANGE = 10

class GraphGUI:
    DEFAULT_PLOT_RANGE_SCALE = 1.25

    def __init__(self, name="",x_label="",y_label=""):
        self.__name = name
        self.__plot = plt
        self.__graph_list = dict()
        self.__ax_position = ('data',0)
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
    def add_graph(self,graph:GPSPlot):
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
    def calibrate_center(self, graph:GPSPlot, graph_type="scatter", zeroed=True):
        
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
        self.__ax_position = ('data',0)
        self.__ay_position = ('data',0)
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
        # self.__ax.xaxis.set_ticks_position('top')
        self.__ax.spines['top'].set_visible(False)
        self.__ax.spines['right'].set_visible(False)

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
    pass

if __name__ == '__main__':
    main()