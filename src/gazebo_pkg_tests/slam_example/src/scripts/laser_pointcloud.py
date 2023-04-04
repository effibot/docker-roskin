#!/home/dock/venvs/py27/bin/python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import qi
import numpy as np
import math


class LaserPointcloud():
    def __init__(self):
        self.sub_front = Subscriber('/cloud', PointCloud2)
        self.sub_left = Subscriber('/cloudl', PointCloud2)
        self.sub_right = Subscriber('/cloudr', PointCloud2)
        self.pub = rospy.Publisher(
            'pointcloud_array', Float32MultiArray, queue_size=10)

    def convert_pointcloud2_to_laserscan(self, pointcloud2):
        # convert pointcloud2 to laser scan decoding the data bytes in little endian and make the new LaserScan message in python2.7
        scan = LaserScan()
        pc2_data = pointcloud2.data
        pc_little_endiand_to_scan = np.frombuffer(
            pc2_data, dtype=np.dtype('<f'))
        print(len(pc_little_endiand_to_scan))
        # conver points in pc_little_endiand_to_scan to Laser ranges
        ranges = []
        for i in range(0, len(pc_little_endiand_to_scan)-4, 4):
            temp_scan = (
                pc_little_endiand_to_scan[i]**2 + pc_little_endiand_to_scan[i+1]**2)**0.5
            angles = math.atan2(
                pc_little_endiand_to_scan[i+1], pc_little_endiand_to_scan[i])
            ranges.append(temp_scan)
        scan.ranges = ranges
        scan.header = pointcloud2.header
        return scan

    def pointcloud_callback(self, sub_front, sub_left, sub_right):
        # print(sub_front.data)
        # convert pointcloud2 to laser scan
        sub_front_scan = self.convert_pointcloud2_to_laserscan(sub_front)
        sub_left_scan = self.convert_pointcloud2_to_laserscan(sub_left)
        sub_right_scan = self.convert_pointcloud2_to_laserscan(sub_right)
        all = [sub_front_scan.ranges,
               sub_left_scan.ranges, sub_right_scan.ranges]
        # print(len(sub_left_scan.ranges))
        # print(sub_front)

    def run(self):
        ats = ApproximateTimeSynchronizer(
            [self.sub_front, self.sub_left, self.sub_right], queue_size=1, slop=0.1)
        ats.registerCallback(self.pointcloud_callback)


def main(args=None):
    rospy.init_node('laser_pointcloud')
    laser_cloud = LaserPointcloud()
    laser_cloud.run()
    rospy.spin()


if __name__ == '__main__':
    main()
