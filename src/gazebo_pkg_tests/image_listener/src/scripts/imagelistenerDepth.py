#!/home/dock/venvs/py27/bin/python
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np


class imagelistener:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/pepper/camera/depth/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

            cv_image_array = np.array(cv_image, dtype=np.dtype('f8'))
            cv_image_norm = cv2.normalize(
                cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            cv2.imshow("Image window", cv_image_norm)
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
