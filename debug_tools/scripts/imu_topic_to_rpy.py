#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rollPublisher = None
yawPublisher = None
pitchPublisher = None

def callback(msg):
    (r, p, y) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    rollPublisher.publish(r)
    pitchPublisher.publish(p)
    yawPublisher.publish(y)

def main():
    rospy.init_node('imu_topic_to_rpy')
    global rollPublisher, yawPublisher, pitchPublisher
    rollPublisher = rospy.Publisher('~roll', Float64, queue_size=1000)
    yawPublisher = rospy.Publisher('~yaw', Float64, queue_size=1000)
    pitchPublisher = rospy.Publisher('~pitch', Float64, queue_size=1000)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.loginfo("Script started!")
    rospy.spin()

if __name__ == '__main__':
    main()