#!/home/dock/venvs/py27/bin/python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import time


class OdomNode(object):

    def __init__(self):
        self.pos_x = None
        self.pos_y = None
        self.orien_z = None
        self.dt_seconds = None
        self.pub = rospy.Publisher(
            '/odom_derivatives', Float32MultiArray, queue_size=1)

    def odometry_callback(self, data):
        # compute (dxy_mm, dtheta_degrees, dt_seconds) from odometry
        if self.pos_x is None and self.pos_y is None and self.orien_z is None:
            self.pos_x = data.pose.pose.position.x
            self.pos_y = data.pose.pose.position.y
            self.orien_z = data.pose.pose.orientation.z
            self.dt_seconds = data.header.stamp
            print(self.dt_seconds)

        else:
            dt = data.header.stamp - self.dt_seconds
            dx = data.pose.pose.position.x - self.pos_x
            dy = data.pose.pose.position.y - self.pos_y
            dxy_mm = [dx*1e3, dy*1e3]
            dtheta_rad = data.pose.pose.orientation.z - self.orien_z
            dtheta_degrees = dtheta_rad * 180 / math.pi
            print("dxy_mm:", dxy_mm)
            print("dtheta_degrees:", dtheta_degrees)
            print("dt_seconds:", dt.to_sec())

            self.dt_seconds = data.header.stamp
            self.pos_x = data.pose.pose.position.x
            self.pos_y = data.pose.pose.position.y
            self.orien_z = data.pose.pose.orientation.z
            self.publish([dxy_mm, dtheta_degrees, dt.to_sec()])

    def publish(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.pub.publish(msg)

    def run(self):
        rospy.init_node('odometry_node')
        rospy.Subscriber('/odom', Odometry,
                         callback=self.odometry_callback)
        rospy.spin()


if __name__ == '__main__':
    node = OdomNode()
    node.run()
