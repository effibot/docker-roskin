#!/home/dock/venvs/py27/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class PathPlanner(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.map = rospy.Subscriber('/rtabmap/scan_map', Image, self.path, queue_size=10)
        pass

    def path(self, map_msgs):
        # convert data.data to array
        cv_image =self.bridge.imgmsg_to_cv2(map_msgs, desired_encoding="passthrough")



if __name__=="__main__":
    rospy.init_node("path_planner")
    path_planner = PathPlanner()
    rospy.spin()