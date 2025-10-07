import pandas as pd
import numpy as np
from scipy.optimize import least_squares


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
        self.__lobf_m = 0
        self.__lobf_b = 0
        self.__draw_lobf = False
        self.__lobf_color = 'black'
        self.__y_independent = False
        self.__slope_data = None
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

##############################################################################################
    
    def calculate_LOBF(self, zeroed=False):
        #calculate slope:

        #Nested functions used for scipy least squares:
        def __model(x, model_params):
            m = model_params[0]
            b = model_params[1]
            y =  m * x + b
            y = np.array(y)
            return y
        def __residual(initial_guesses, x, y):
            aaa= (y - __model(x, initial_guesses)).flatten()
            return aaa

        #calculate slope:
        if not zeroed:
            x = self.__data[self.__x_axis_field] - self.__x_mean
            y = self.__data[self.__y_axis_field] - self.__y_mean
        else:
            x = self.__data[self.__x_axis_zeroed] - self.__x_mean
            y = self.__data[self.__y_axis_zeroed] - self.__y_mean

        guesses = [0,0]
        lsq_min = least_squares(__residual, guesses , args=(x.values, y.values))
        model_vals = lsq_min.x
        self.__lobf_m = model_vals[0]
        self.__lobf_b = model_vals[1]

        self.__draw_lobf = True

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
        elif option.upper() == "HIST":
            self.__data[self.__x_axis_zeroed ] = self.__data[self.__x_axis_field] - self.__x_mean
            self.__data[self.__y_axis_zeroed] = self.__data[self.__y_axis_field] - self.__y_mean
            pass
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
    def get_lobf_slope(self):
        return self.__lobf_m
    def get_lobf_b(self):
        return self.__lobf_b
    def get_lobf_draw_state(self):
        return self.__draw_lobf
    def get_lobf_color(self):
        return self.__lobf_color
    def get_y_independent(self):
        return self.__y_independent
    def get_slope_data(self):
        return self.__slope_data
    
    #Setters:
    def set_x_range(self,value):
        self.__x_range = value
    def set_y_range(self,value):
        self.__y_range = value
    def set_lobf_draw_state(self, value):
        self.__draw_lobf=value
    def set_lobf_color(self,value):
        self.__lobf_color = value
    def set_y_independent(self,value):
        self.__y_independent=value
    

##############################################################################################  
##############################################################################################  
def main():
    pass

if __name__ == '__main__':
    main()