#!/home/dock/venvs/py27/bin/python


from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from breezyslam.sensors import Laser
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from breezyslam.algorithms import RMHC_SLAM
import cv2
from PIL import Image
import numpy as np
import struct
import yaml
from std_msgs.msg import Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import matplotlib.pyplot as plt

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 6


class SLAMNode(object):
    def __init__(self):
        print("OK")
        self.laser = Laser(45, 6.66, 240, 5400, 0, 100)
        self.slam = RMHC_SLAM(self.laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        self.pose_change = [0, 0, 0]
        self.pose_pub = None
        self.map_pub = None
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    def display_map(self):
        # display slam map in opencv window
        self.slam.getmap(self.mapbytes)
        # img = Image.frombytes(
        #    'L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), bytes(self.mapbytes))
        # img = np.array(img)
        cv2.imshow('SLAM', np.array(Image.frombytes(
            'L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), bytes(self.mapbytes))))
        cv2.waitKey(1)

    def slam_callback(self, scan, odom):
        odom_pose = (odom.data[0]**2 + odom.data[1]**2)**0.5
        odom_orient = odom.data[2]
        odom_time = odom.data[3]

        self.slam.update(list(scan.data), pose_change=[odom_pose, odom_orient, odom_time],
                         scan_angles_degrees=list(np.linspace(-120, 120, 45)), should_update_map=True)
        self.display_map()

    def run(self):
        rospy.init_node('slam_node')
        lidar_sub = Subscriber('/lidar_array',
                               Float32MultiArray)
        odom_sub = Subscriber("/odom_derivatives",
                              Float32MultiArray)
        ats = ApproximateTimeSynchronizer(
            [lidar_sub, odom_sub], queue_size=1, slop=0.1, allow_headerless=True)
        ats.registerCallback(self.slam_callback)
        rospy.spin()


if __name__ == '__main__':
    node = SLAMNode()
    print("Starting SLAM node...")
    node.run()
