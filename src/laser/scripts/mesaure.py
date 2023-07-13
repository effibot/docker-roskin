import rospy
from sensor_msgs.msg import LaserScan
class Laser_Mean():
    def __init__(self):
        self.laser_sub = rospy.Subscriber("/pepper_robot/laser", LaserScan, self.compute_metrics)
        self.range_min = 0
        self.range_max = 0
        self.counter = 20
        self.batch = []
        self.scan_time = 0
    def compute_metrics(self, laser_data):
        self.scan_time = laser_data.time_increment
        self.range_min = laser_data.range_min
        self.range_max = laser_data.range_max
        self.ranges = laser_data.ranges

    def run(self):

        while rospy.is_shutdown():
            if len(self.batch) == self.counter:


if __name__=="__main__":
    rospy.init_node("Laser Values")
    obj = Laser_Mean()
    obj.run()
    rospy.spin()