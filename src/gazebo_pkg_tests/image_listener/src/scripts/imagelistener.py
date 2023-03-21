#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class imagelistener:
    """
    It creates a subscriber to the topic /pepper/camera/front/image_raw, which is the topic that
    publishes the images from the front camera of the robot. The subscriber is created with the function
    image_sub = rospy.Subscriber("/pepper/camera/front/image_raw", Image, self.callback). The first
    argument is the topic name, the second is the type of message that is published on the topic, and
    the third is the callback function that is called when a message is received
    
    :param args: This is a list of command line arguments
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/pepper/camera/front/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)


def main(args):
    il = imagelistener()
    rospy.init_node('imagelistener', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
