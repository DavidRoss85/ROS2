import pandas as pd

##############################################################################################  
##############################################################################################  

class GPSPlot:
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

    def __calculate_euclidean(self,x1,y1,x2,y2):
        xdiff = x1-x2
        xsqr = xdiff**2
        ydiff = y1-y2
        ysqr = ydiff**2
        sqr_sum = xsqr+ysqr
        dist = (((x1-x2)**2) + ((y1-y2)**2))**0.5
        return dist
##############################################################################################  
    def generate_euclidean(self):
        self.__data['EUCLIDEAN'] = self.__calculate_euclidean(
            self.__data[self.__x_axis_field],
            self.__data[self.__y_axis_field],
            self.__x_mean,
            self.__y_mean
        )
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
            
        return
                
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
def main():
    pass

if __name__ == '__main__':
    main()