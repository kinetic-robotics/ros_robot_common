#!/usr/bin/env python3

import rospy
from rm_referee_ui_framework.manager import Manager
from rm_referee_ui_framework.widgets import arcWidget, lineWidget

rospy.init_node("test_ui", anonymous=True)
uiManager = Manager()
uiManager.init(1, "/ui")
rospy.sleep(3)
arc1 = arcWidget.ArcWidget(startAngle = 3.0)
arc1.relativeX = 30
arc1.relativeY = 40
arc2 = arcWidget.ArcWidget(startAngle = 4.0)
arc2.relativeX = 50
arc2.relativeY = 60
line = lineWidget.LineWidget()
line.relativeX = 70
line.relativeY = 80
line.endX = 70
line.endY = 80
arc1.add(arc2)
uiManager.add(arc1)
uiManager.add(line)
uiManager.update()
uiManager.update()
arc1.hide = True
arc1.relativeX = 100
uiManager.update()
uiManager.update()
rospy.spin()