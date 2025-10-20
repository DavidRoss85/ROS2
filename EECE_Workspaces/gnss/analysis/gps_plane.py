import matplotlib.pyplot as plt
import numpy as np
from gps_graph import GPSPlot

DEFAULT_RANGE = 1

class GraphGUI:
    DEFAULT_PLOT_RANGE_SCALE = 1.25
    DEFAULT_PADDING_SCALE = 0.25
    DEFAULT_MIN_PADDING = 1e-12

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
        elif graph_type.upper() == "HIST":
            pass
        elif graph_type.upper() == "ALLAN":
            pass
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
            if graph.get_lobf_draw_state():
                gx1= self.__x_center - self.__x_range
                gx2= self.__x_center + self.__x_range
                gm = graph.get_lobf_slope()
                gb = graph.get_lobf_b()
                gy1=gm*gx1 + gb
                gy2=gm*gx2 + gb


                self.__plot.plot(
                    [gx1,gx2],
                    [gy1,gy2],
                    color=graph.get_lobf_color(),
                    label=f'Line of best fit: y={gm:.2}x+{gb:.2}'
                )


        # Set chart properties: place spines at the current data center so 0 values align when zeroed
        self.__ax_position = ('data', self.__x_center)
        self.__ay_position = ('data', self.__y_center)
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
            self.__plot.hist(
                # graph.get_data()[graph.get_x_zeroed_field() if zeroed else graph.get_x_axis_field()], 
                graph.get_data()[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()],
                color=graph.get_color(),
                alpha=graph.get_alpha(),
                label=graph.get_name(),
                edgecolor='black',
                bins=30
            )

            self.calibrate_center(graph,"histogram",zeroed)
        #Set chart properties:
        self.__ax_position = ('data',self.__x_min)
        self.__ay_position = ('data',self.__y_min)


##############################################################################################        
    def apply_line_settings(self,zeroed=True):
        for graph in self.__graph_list.values():
            self.__plot.plot(
                graph.get_data()[graph.get_x_zeroed_field() if zeroed else graph.get_x_axis_field()], 
                graph.get_data()[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()],
                color=graph.get_color(),
                alpha=graph.get_alpha(),
                label=graph.get_name()
            )
            if graph.get_fill_color() != 'None':
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
    def apply_allan_settings(self,zeroed=False):
        # Collect min/max across all Allan series so we can set reasonable log-axis limits
        x_mins = []
        x_maxs = []
        y_mins = []
        y_maxs = []

        for graph in self.__graph_list.values():
            s = graph.get_allan_series()
            try:
                x_mins.append(float(s.index.min()))
                x_maxs.append(float(s.index.max()))
                y_mins.append(float(s.values.min()))
                y_maxs.append(float(s.values.max()))
            except Exception:
                # skip series that cannot be converted
                continue

            # plot using log-log scale
            self.__plot.loglog(
                s.index,
                s.values,
                color=graph.get_color(),
                alpha=graph.get_alpha(),
                label=graph.get_name()
            )

            # no need to calibrate center for Allan plots (they're log-log)

        # Set chart properties: place spines on left and bottom for Allan plots
        # use axes coordinates so the spines are at the left and bottom edges
        self.__ax_position = ('axes', 0.0)   # left spine at axes x=0 (left)
        self.__ay_position = ('axes', 0.0)   # bottom spine at axes y=0 (bottom)

        # If we collected ranges, set limits with a small padding to avoid clipping
        if x_mins and x_maxs and y_mins and y_maxs:
            xmin = min(x_mins)
            xmax = max(x_maxs)
            ymin = min(y_mins)
            ymax = max(y_maxs)

            # If the min is non-positive, prefer the smallest positive value from the collected mins
            pos_xmins = [v for v in x_mins if v is not None and v > 0]
            pos_ymins = [v for v in y_mins if v is not None and v > 0]
            if xmin <= 0:
                xmin = min(pos_xmins) if pos_xmins else self.DEFAULT_MIN_PADDING
            if ymin <= 0:
                ymin = min(pos_ymins) if pos_ymins else self.DEFAULT_MIN_PADDING

            xpad = (xmax / xmin) ** self.DEFAULT_PADDING_SCALE if xmin > 0 else self.DEFAULT_PLOT_RANGE_SCALE
            ypad = (ymax / ymin) ** self.DEFAULT_PADDING_SCALE if ymin > 0 else self.DEFAULT_PLOT_RANGE_SCALE

            self.__plot.xlim(xmin / xpad, xmax * xpad)
            self.__plot.ylim(ymin / ypad, ymax * ypad)

        # Ensure axes use log scale (loglog above usually sets this but be explicit)
        self.__ax.set_xscale('log')
        self.__ax.set_yscale('log')

    def _compute_and_apply_limits(self, graph_type='scatter', zeroed=True, pad_fraction=DEFAULT_PADDING_SCALE, min_padding=DEFAULT_MIN_PADDING):
        """Compute combined axis limits across all graphs for the given graph_type and apply xlim/ylim.
        pad_fraction: fraction of the data range to use as padding on each side.
        """
        x_arrays = []
        y_arrays = []

        for graph in self.__graph_list.values():
            df = graph.get_data()
            try:
                if graph_type.upper() == 'ALLAN':
                    return
                    # s = graph.get_allan_series()
                    # if s is None or len(s) == 0:
                    #     continue
                    # x_arrays.append(np.array(s.index, dtype=float))
                    # y_arrays.append(np.array(s.values, dtype=float))
                elif graph_type.upper() == 'HIST':
                    # Use the data values for x axis limits (histograms are plotted from data values)
                    xvals = df[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()].dropna().values
                    if len(xvals) > 0:
                        x_arrays.append(np.array(xvals, dtype=float))
                else:
                    xvals = df[graph.get_x_zeroed_field() if zeroed else graph.get_x_axis_field()].dropna().values
                    yvals = df[graph.get_y_zeroed_field() if zeroed else graph.get_y_axis_field()].dropna().values
                    if len(xvals) > 0:
                        x_arrays.append(np.array(xvals, dtype=float))
                    if len(yvals) > 0:
                        y_arrays.append(np.array(yvals, dtype=float))
            except Exception:
                # skip graphs with incompatible data
                continue

        # If we have no data, do nothing
        if not x_arrays and not y_arrays:
            return

        # Compute combined min/max
        if x_arrays:
            all_x = np.concatenate(x_arrays)
            all_x = all_x[np.isfinite(all_x)]
            if all_x.size > 0:
                xmin = float(np.min(all_x))
                xmax = float(np.max(all_x))
            else:
                xmin, xmax = None, None
        else:
            xmin, xmax = None, None

        if y_arrays:
            all_y = np.concatenate(y_arrays)
            all_y = all_y[np.isfinite(all_y)]
            if all_y.size > 0:
                ymin = float(np.min(all_y))
                ymax = float(np.max(all_y))
            else:
                ymin, ymax = None, None
        else:
            ymin, ymax = None, None

        # Helper to compute padded limits
        def _padded_limits(vmin, vmax):
            if vmin is None or vmax is None:
                return None, None
            if np.isclose(vmin, vmax):
                # Expand a single-value range
                if abs(vmin) > 1e-9:
                    delta = abs(vmin) * 0.05
                else:
                    delta = 1.0
                return vmin - delta, vmax + delta
            rng = vmax - vmin
            pad = max(rng * pad_fraction, min_padding)
            return vmin - pad, vmax + pad

        xlim = _padded_limits(xmin, xmax)
        ylim = _padded_limits(ymin, ymax)

        # If this is a centered scatter plot, enforce symmetric limits about the center
        if graph_type.upper() == 'SCATTER' and zeroed:
            # Use the stored centers (set by calibrate_center). Fall back to 0 if unset.
            cx = getattr(self, '_GraphGUI__x_center', self.__x_center if hasattr(self, '_GraphGUI__x_center') else 0)
            cy = getattr(self, '_GraphGUI__y_center', self.__y_center if hasattr(self, '_GraphGUI__y_center') else 0)

            # Compute max absolute deviations
            try:
                if x_arrays:
                    all_x = np.concatenate(x_arrays)
                    all_x = all_x[np.isfinite(all_x)]
                    if all_x.size > 0:
                        max_abs_x = float(np.max(np.abs(all_x - cx)))
                        # pad and set symmetric limits
                        pad = max(max_abs_x * pad_fraction, min_padding)
                        xlim = (cx - max_abs_x - pad, cx + max_abs_x + pad)
                if y_arrays:
                    all_y = np.concatenate(y_arrays)
                    all_y = all_y[np.isfinite(all_y)]
                    if all_y.size > 0:
                        max_abs_y = float(np.max(np.abs(all_y - cy)))
                        pad = max(max_abs_y * pad_fraction, min_padding)
                        ylim = (cy - max_abs_y - pad, cy + max_abs_y + pad)
            except Exception:
                # If something goes wrong, fall back to prior behavior
                pass

        # Apply limits carefully for log scales (can't include zeros or negatives)
        try:
            xscale = self.__ax.get_xscale()
            yscale = self.__ax.get_yscale()
        except Exception:
            xscale = 'linear'
            yscale = 'linear'

        if xlim[0] is not None and xlim[1] is not None:
            x0, x1 = xlim
            if xscale == 'log':
                x0 = max(x0, min_padding)
                x1 = max(x1, x0 * 1.0001)
            self.__plot.xlim(x0, x1)

        if ylim[0] is not None and ylim[1] is not None:
            y0, y1 = ylim
            if yscale == 'log':
                y0 = max(y0, min_padding)
                y1 = max(y1, y0 * 1.0001)
            self.__plot.ylim(y0, y1)

##############################################################################################        
    def show(self,graph_type='scatter', zeroed=True):
        """
        Cycle through all graphs and plot with varying colors
        :return:
        """
        if len(self.__graph_list)==0: return  #Exit if no graphs in dictionary

        if graph_type.upper() == 'SCATTER':
            self.apply_scatterplot_settings(zeroed)
        elif graph_type.upper() == 'HIST':
            self.apply_histogram_settings(zeroed)
        elif graph_type.upper() == 'ALLAN':
            self.apply_allan_settings(zeroed)
        else:
            self.apply_line_settings(zeroed)
        # Compute and apply axis limits consistently across graph types
        self._compute_and_apply_limits(graph_type, zeroed)
            

        #Set chart properties:
        self.__ax.spines['left'].set_position(self.__ax_position)
        self.__ax.spines['bottom'].set_position(self.__ay_position)
        # self.__ax.xaxis.set_ticks_position('top')
        self.__ax.spines['top'].set_visible(False)
        self.__ax.spines['right'].set_visible(False)

        self.__plot.legend(loc="upper right")
        self.__plot.title(self.__name)
        # self.__plot.ylabel(self.__y_label)
        # self.__plot.xlabel(self.__x_label)
        self.__ax.set_xlabel(self.__x_label, loc='right', labelpad=10)
        self.__ax.set_ylabel(self.__y_label, loc='top',labelpad=10)
        
        
        

        #Display chart
        self.__plot.show()

##############################################################################################  
    #Setters:
    def set_x_axis_position(self, value:str):
        self.__ax_position = value
    def set_y_axis_position(self, value:str):
        self.__ay_position = value
    def set_x_range(self, value):
        self.__x_range = value
    def set_y_range(self, value):
        self.__y_range = value

##############################################################################################  
##############################################################################################  
def main():
    pass

if __name__ == '__main__':
    main()