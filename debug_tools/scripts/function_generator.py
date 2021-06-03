#!/usr/bin/env python3

import math, argparse, matplotlib, rospy, numpy

from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent

matplotlib.rcParams["toolbar"] = "none"

class DraggablePlotExample(object):
    u""" An example of plot with draggable markers """

    def __init__(self, minX = 0.0, minY = 0.0, maxX = 1.0, maxY = 1.0, param = "", read = False):
        self._figure, self._axes, self._line = None, None, None
        self._dragging_point = None
        self._points = {}
        self._pointsABS = {}
        self.__minX, self.__minY = minX, minY
        self.__maxX, self.__maxY = maxX, maxY
        self.__param = param
        if param != "":
            dataX, dataY = rospy.get_param(self.__param + "/x"), rospy.get_param(self.__param + "/y")
            if read:
                self.__minX, self.__minY = dataX[0],  dataY[0]
                self.__maxX, self.__maxY = dataX[-1], dataY[-1]
            for i in range(len(dataX)):
                if not ((dataX[i] == self.__minX and dataY[i] == self.__minY) or (dataX[i] == self.__maxX and dataY[i] == self.__maxY)):
                    self._points[dataX[i]] = dataY[i]
            self.__update_service = rospy.ServiceProxy(self.__param + "/update_config", Trigger)
        self._init_plot()

    def _init_plot(self):
        self._figure = plt.figure("Function Generator")

        axes = plt.subplot(1, 1, 1)
        axes.set_xlim(self.__minX, self.__maxX)
        axes.set_ylim(self.__minY, self.__maxY)
        axes.xaxis.set_major_locator(plt.MultipleLocator((self.__maxX - self.__minX) / 10))
        axes.yaxis.set_major_locator(plt.MultipleLocator((self.__maxY - self.__minY) / 10))
        plt.axvline(0)
        plt.axhline(0)
        axes.grid(which="both")
        self._axes = axes

        self._figure.canvas.mpl_connect('button_press_event', self._on_click)
        self._figure.canvas.mpl_connect('button_release_event', self._on_release)
        self._figure.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self._update_plot()
        plt.show()

    def _update_plot(self):
        items = list(self._points.items())
        items.append((self.__minX, self.__minY))
        items.append((self.__maxX, self.__maxY))
        x, y = zip(*sorted(items))
        # Add new plot
        if not self._line:
            self._line, = self._axes.plot(x, y, "b", marker="o", markersize=10, markevery=slice(1, -1))
        # Update current plot
        else:
            self._line.set_data(x, y)
        xy_pixels = self._axes.transData.transform(numpy.vstack([x[1:-1],y[1:-1]]).T)
        xpix, ypix = xy_pixels.T
        i = 0
        for xs in x[1:-1]:
            self._pointsABS[xs] = (xpix[i], ypix[i])
            i = i + 1
        self._figure.canvas.draw()
        if self._points:
            outputX = [str(out) for out in x]
            outputY = [str(out) for out in y]
            print("x: [", ", ".join(outputX), "]")
            print("y: [", ", ".join(outputY), "]")
            print()
            if self.__param != "":
                paramX = [float(var) for var in x]
                paramY = [float(var) for var in y]
                rospy.set_param(self.__param + "/x", paramX)
                rospy.set_param(self.__param + "/y", paramY)
                self.__update_service()

    def _add_point(self, x, y=None):
        if isinstance(x, MouseEvent):
            targetX, targetY = round(x.xdata, 2), round(x.ydata, 2)
        self._points[targetX] = targetY
        return targetX, targetY

    def _remove_point(self, x, _):
        if x in self._points:
            self._points.pop(x)
            self._pointsABS.pop(x)


    def _find_neighbor_point(self, event):
        u""" Find point around mouse position

        :rtype: ((int, int)|None)
        :return: (x, y) if there are any point around mouse else None
        """
        distance_threshold = 0.1 * self._figure.dpi
        nearest_point = None
        min_distance = 100 ** 2
        for x, y in self._pointsABS.items():
            distance = math.hypot(event.x - y[0], event.y - y[1])
            if distance < min_distance:
                min_distance = distance
                nearest_point = (x, self._points[x])
        if min_distance < distance_threshold:
            return nearest_point
        return None

    def _on_click(self, event):
        u""" callback method for mouse click event

        :type event: MouseEvent
        """
        # left click
        if event.button == 1 and event.inaxes in [self._axes]:
            point = self._find_neighbor_point(event)
            if point:
                self._dragging_point = point
            else:
                self._add_point(event)
            self._update_plot()
        # right click
        elif event.button == 3 and event.inaxes in [self._axes]:
            point = self._find_neighbor_point(event)
            if point:
                self._remove_point(*point)
                self._update_plot()

    def _on_release(self, event):
        u""" callback method for mouse release event

        :type event: MouseEvent
        """
        if event.button == 1 and event.inaxes in [self._axes] and self._dragging_point:
            self._dragging_point = None
            self._update_plot()

    def _on_motion(self, event):
        u""" callback method for mouse motion event

        :type event: MouseEvent
        """
        if not self._dragging_point:
            return
        if event.xdata is None or event.ydata is None:
            return
        self._remove_point(*self._dragging_point)
        self._dragging_point = self._add_point(event)
        self._update_plot()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='function generate.')
    parser.add_argument('--param',type=str,   default="",  help='Parameters path of robot_toolbox')
    parser.add_argument('--minx', required=False, type=float, default=0.0, help='Min X shows in graph.')
    parser.add_argument('--miny', required=False, type=float, default=0.0, help='Min Y shows in graph.')
    parser.add_argument('--maxx', required=False, type=float, default=1.0, help='Max X shows in graph.')
    parser.add_argument('--maxy', required=False, type=float, default=1.0, help='Max Y shows in graph.')
    parser.add_argument('--read', required=False, type=int,   default=0,   help='Read parameters from server, if it is 1, minx, miny, maxx, maxy will be ignored!')
    args = parser.parse_args()
    rospy.init_node("function_generator")
    print(bool(args.read))
    plot = DraggablePlotExample(args.minx, args.miny, args.maxx, args.maxy, args.param, bool(args.read))