#!/home/dock/venvs/py27/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from breezyslam.sensors import Laser
from std_msgs.msg import Float32MultiArray
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber


class Pepper_Lidar(Laser):
    def __init__(self):
        Laser.__init__(self, 15, 10, 240, 5600, 0, 20)
        self.laser_array = []
        self.publisher = rospy.Publisher(
            'lidar_array', Float32MultiArray, queue_size=1)

    def laser_callback(self, left, front, right):
        self.laser_array = np.asarray(
            [left.ranges, front.ranges, right.ranges]).flatten()
        self.publish()
        self.laser_array = []

    def run(self):
        rospy.init_node('pepper_lidar_node')
        left_sub = Subscriber('/laser/srd_left/scan',
                              LaserScan)
        front_sub = Subscriber('/laser/srd_front/scan',
                               LaserScan)
        right_sub = Subscriber('/laser/srd_right/scan',
                               LaserScan)
        ats = ApproximateTimeSynchronizer(
            [left_sub, front_sub, right_sub], queue_size=1, slop=0.1)
        ats.registerCallback(self.laser_callback)
        rospy.spin()

    def publish(self):
        msg = Float32MultiArray()
        msg.data = self.laser_array
        print("Message", len(self.laser_array))
        self.publisher.publish(msg)


if __name__ == '__main__':
    Pepper_Lidar().run()
