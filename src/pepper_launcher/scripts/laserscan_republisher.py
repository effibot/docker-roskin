"""
    This script republishes the laserscan topic from the Pepper robot.
    We need to republish the laserscan topic because the /naoqi_driver/laser topic
    publish too few points to be useful for a mapping algorithm.
"""
#! ~/venvs/py27/bin/python

import rospy
from sensor_msgs.msg import LaserScan

# Define the subscriber class to get the laserscan data from the robot
class laser_sub():
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/naoqi_driver/laser', LaserScan, self.laser_callback)

    def laser_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.ranges)

def main():
    rospy.init_node('laserscan_republisher', anonymous=True)
    laser_sub()
    rospy.spin()

if __name__ == '__main__':
    main()