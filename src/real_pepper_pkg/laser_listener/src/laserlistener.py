#!/home/dock/venvs/py27/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
# Class that takes all sensor value from front/back/left/right laser


class LaserListener:

    def __init__(self):

        # Ground Laser
        rospy.Subscriber("/laser/ground_left/scan",
                         LaserScan, self.laser_fground_leftcallback)
#        rospy.Subscriber("/laser/ground_right/scan",
#                         LaserScan, self.laser_fground_rightcallback)
#
#        # Ground Laser pointcloud
#        rospy.Subscriber("/laser/ground_left/pointcloud",
#                         PointCloud, self.laser_fground_leftPointcallback)
#        rospy.Subscriber("/laser/ground_right/pointcloud",
#                         PointCloud, self.laser_fground_rightPointcallback)
#
#        # SRD Laser
#        rospy.Subscriber("/laser/srd_front/scan",
#                         LaserScan, self.laser_srd_frontcallback)
#        rospy.Subscriber("/laser/srd_left/scan",
#                         LaserScan, self.laser_srd_leftcallback)
#        rospy.Subscriber("/laser/srd_right/scan",
#                         LaserScan, self.laser_srd_right_callback)
#
#        # SRD Laser pointcloud
#        rospy.Subscriber("/laser/srd_front/pointcloud",
#                         PointCloud, self.laser_srd_frontPointcallback)
#        rospy.Subscriber("/laser/srd_left/pointcloud",
#                         PointCloud, self.laser_srd_leftPointcallback)
#        rospy.Subscriber("/laser/srd_right/pointcloud",
#                         PointCloud, self.laser_srd_rightPointcallback)

    def laser_fground_leftcallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)

    # def laser_fground_rightcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
#
    # def laser_fground_leftPointcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.points)
#
    # def laser_fground_rightPointcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.points)
#
    # def laser_srd_frontcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
#
    # def laser_srd_leftcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
#
    # def laser_srd_right_callback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
#
    # def laser_srd_frontPointcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.points)
#
    # def laser_srd_leftPointcallback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.points)
#
    # def laser_srd_rightPointcallback(self, data):
    #    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.points)


if __name__ == '__main__':
    ls = LaserListener()
    rospy.init_node('laser_listener', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
