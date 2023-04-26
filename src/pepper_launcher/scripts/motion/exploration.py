#!/home/dock/venvs/py27/bin/python
import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan

class exploration():
    def __init__(self) -> None:
        laser_srd = rospy.Subscriber("/naoqi_driver/laser",LaserScan)





if __name__=='main':
    exp_node = exploration()
    rospy.init_node("exploration")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
