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


MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 5


class SLAMNode(object):
    def __init__(self):
        print("OK")
        self.laser = Laser(45, 10, 120, 5600, 0, 0)
        self.slam = RMHC_SLAM(self.laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        self.pose_change = [0, 0, 0]
        self.pose_pub = None
        self.map_pub = None

    def display_map(self):
        # display slam map in opencv window
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.slam.getmap(mapbytes)
        img = Image.frombytes(
            'L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), bytes(mapbytes))
        img = np.array(img)
        cv2.imshow('SLAM', img)
        cv2.waitKey(1)

    def scan_callback(self, data):
        self.slam.update(
            list(data.data), scan_angles_degrees=list(np.linspace(-120, 120, 45)))
        self.display_map()

    def odom_callback(self, data):
        self.pose_change = data.data
        self.slam.update(
            self.pose_change[0], self.pose_change[1], self.pose_change[2])

    def run(self):
        rospy.init_node('slam_node')
        rospy.Subscriber('/lidar_array',
                         Float32MultiArray, self.scan_callback)
        rospy.Subscriber("odom_derivatives",
                         Float32MultiArray, self.odom_callback)
        # self.pose_pub = rospy.Publisher(
        #    '/slam/pose', PoseStamped, queue_size=10)
        # self.map_pub = rospy.Publisher(
        #    '/slam/map', OccupancyGrid, queue_size=10)
        rospy.spin()


if __name__ == '__main__':
    node = SLAMNode()
    print("Starting SLAM node...")
    node.run()
