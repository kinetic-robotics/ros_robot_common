#!/usr/bin/env python3

import rospy
from rm_referee_ui_framework.color import ColorType
from rm_referee_ui_framework.manager import Manager
from rm_referee_ui_framework.widgets import arcWidget, lineWidget, stringWidget

rospy.init_node("test_ui", anonymous=True)
uiManager = Manager()
uiManager.init(1, "/rm_referee_controller/ui/update", "/rm_referee_controller/ui/delete")
rospy.sleep(1)
string = stringWidget.StringWidget()
string.relativeX = 600
string.relativeY = 500
string.color = ColorType.WHITE
string.fontSize = 100
string.width = 10
string.data = "1"
uiManager.add(string)
string.data = str(int(string.data) + 1)
uiManager.update()