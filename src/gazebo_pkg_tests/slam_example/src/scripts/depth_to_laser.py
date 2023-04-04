#!/home/dock/venvs/py27/bin/python
import rospy
from PIL import Image as im
from sensor_msgs.msg import Image, LaserScan
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
import numpy as np
import cv2
import struct
from message_filters import TimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
import math
import time

MAP_SIZE_PIXELS = 320
MAP_SIZE_METERS = 1


class DepthToLaserConverter:
    def __init__(self):
        # create instance of DepthImageToLaserScan class
        self.laser = Laser(320, 20, 58, 6200, 0, 0)
        self.slam = RMHC_SLAM(self.laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        self.pose_change = [0, 0, 0]
        self.pose_pub = None
        self.map_pub = None
        self.scan = Subscriber(
            '/scan', LaserScan)
        odom_sub = Subscriber("/pepper/odom",
                              Odometry)
        ats = TimeSynchronizer(
            [self.scan, odom_sub], queue_size=1)
        ats.registerCallback(self.depth_callback)
        self.currx = 0
        self.curry = 0
        self.currz = 0
        self.dtheta = 0
        self.dt = 0
        self.i = 0

    def depth_callback(self, depth_image, odom):
        # print("odom", odom)
        # print("depth_image_lenght", len(depth_image.ranges))
        dxy = ((odom.pose.pose.position.x*1000-self.currx)**2 +
               (odom.pose.pose.position.y*1000-self.curry)**2)**0.5
        self.currx = odom.pose.pose.position.x
        self.curry = odom.pose.pose.position.y
        self.dtheta = math.degrees(odom.pose.pose.orientation.z)-self.dtheta
        self.dt = odom.header.stamp.secs-self.dt
        # print("pose_change", [(self.currx**2+self.curry**2)
        #      ** 0.5, math.degrees(self.dtheta), self.dt])
        # multiply the array values by 1000 in the depth_image.ranges
        scaled = np.array(depth_image.ranges)*1000
        # print("pose_change", [dxy, self.dtheta, self.dt])
        # print("scaled", scaled)
        self.slam.update(scans_mm=list(scaled), pose_change=list(
            [dxy, self.dtheta, self.dt]), scan_angles_degrees=list(np.linspace(-58/2, 58/2, 320)),  should_update_map=True)
        # show slam map
        # cv2.namedWindow('SLAM', cv2.WINDOW_NORMAL)
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.slam.getmap(mapbytes)
        cv2.imshow('SLAM', np.array(im.frombytes(
            'L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), bytes(mapbytes))))
        cv2.waitKey(0)
        self.i = self.i+1
        print(self.i)


if __name__ == '__main__':
    rospy.init_node('depth_to_laser_converter')
    converter = DepthToLaserConverter()
    rospy.spin()
