#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent

class DraggablePlotExample(object):
    u""" An example of plot with draggable markers """

    def __init__(self):
        self._figure, self._axes, self._line = None, None, None
        self._dragging_point = None
        self._points = {}

        self._init_plot()

    def _init_plot(self):
        self._figure = plt.figure("Function Generator")
        self._figure.canvas.toolbar.pack_forget()

        axes = plt.subplot(1, 1, 1)
        axes.set_xlim(0, 1)
        axes.set_ylim(0, 1)
        axes.xaxis.set_major_locator(plt.MultipleLocator(0.1))
        axes.yaxis.set_major_locator(plt.MultipleLocator(0.1))
        axes.grid(which="both")
        self._axes = axes

        self._figure.canvas.mpl_connect('button_press_event', self._on_click)
        self._figure.canvas.mpl_connect('button_release_event', self._on_release)
        self._figure.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self._update_plot()
        plt.show()

    def _update_plot(self):
        items = self._points.items()
        items.insert(0, (0, 0))
        items.append((1, 1))
        x, y = zip(*sorted(items))
        # Add new plot
        if not self._line:
            self._line, = self._axes.plot(x, y, "b", marker="o", markersize=10, markevery=slice(1, -1))
        # Update current plot
        else:
            self._line.set_data(x, y)
        self._figure.canvas.draw()
        if self._points:
            outputX = [str(out) for out in x][1:-1]
            outputY = [str(out) for out in y][1:-1]
            print("x: [", ", ".join(outputX), "]")
            print("y: [", ", ".join(outputY), "]")
            print()

    def _add_point(self, x, y=None):
        if isinstance(x, MouseEvent):
            x, y = round(x.xdata, 2), round(x.ydata, 2)
        self._points[x] = y
        return x, y

    def _remove_point(self, x, _):
        if x in self._points:
            self._points.pop(x)

    def _find_neighbor_point(self, event):
        u""" Find point around mouse position

        :rtype: ((int, int)|None)
        :return: (x, y) if there are any point around mouse else None
        """
        distance_threshold = 0.03
        nearest_point = None
        min_distance = math.sqrt(2 * (100 ** 2))
        for x, y in self._points.items():
            distance = math.hypot(event.xdata - x, event.ydata - y)
            if distance < min_distance:
                min_distance = distance
                nearest_point = (x, y)
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
    plot = DraggablePlotExample()