#!/home/dock/venvs/py27/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from breezyslam.sensors import Laser
from std_msgs.msg import Float32MultiArray
import numpy as np


class Pepper_Lidar(Laser):
    def __init__(self):
        Laser.__init__(self, 15, 10, 240, 5600, 0, 0)
        self.laser_left = []
        self.laser_front = []
        self.laser_right = []
        self.publisher = rospy.Publisher(
            'lidar_array', Float32MultiArray, queue_size=1)

    def laser_callback(self, data, arg):
        if arg == 'laser_left':
            self.laser_left = data.ranges
        elif arg == 'laser_front':
            self.laser_front = data.ranges
        else:
            self.laser_right = data.ranges
        if self.laser_left and self.laser_front and self.laser_right:
            self.publish()
            self.laser_left = []
            self.laser_front = []
            self.laser_right = []

    def run(self):
        rospy.init_node('pepper_lidar_node')
        rospy.Subscriber('/pepper/scan_left',
                         LaserScan, callback=self.laser_callback, callback_args='laser_left')
        rospy.Subscriber('/pepper/scan_front',
                         LaserScan, callback=self.laser_callback, callback_args='laser_front')
        rospy.Subscriber('/pepper/scan_right',
                         LaserScan, callback=self.laser_callback, callback_args='laser_right')
        rospy.spin()

    def publish(self):
        data = []
        data.append(self.laser_left)
        data.append(self.laser_front)
        data.append(self.laser_right)
        # array of list to array 1D
        data = np.array(data).flatten()
        msg = Float32MultiArray()
        msg.data = data
        self.publisher.publish(msg)


if __name__ == '__main__':
    Pepper_Lidar().run()
