#!/home/dock/venvs/py27/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def produce():
    ranges = [0.45477494597435, 0.739227294921875, 0.7411246299743652, 0.6374923586845398, 0.6099849939346313, 0.610636293888092, 0.67930668592453, 1.2298823595046997, 0.7874770164489746, 0.8490968942642212, 0.8214210271835327, 1.3202682733535767, 1.3532896041870117, 1.350544810295105, 1.38477623462677, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 7.050281047821045, 7.051639080047607, 7.052820205688477, 7.053819179534912, 1.3611520528793335, 1.3617579936981201, 3.186048746109009, 7.055942058563232, 1.4020845890045166, 7.055856704711914, 7.055526256561279, 7.055004596710205, 0.6873072981834412, 0.6963121294975281, 0.7158398032188416, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.5097943544387817, 1.5606811046600342, 0.6713876128196716, 0.5920705199241638, 0.5750609040260315, 0.5827099084854126, 1.797183871269226, 1.7979685068130493, 1.7983046770095825, 7.091649055480957, 0.7240342497825623, 0.723102867603302, 0.44776394963264465, 0.438874751329422, 0.47583523392677307]
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.angle_min = -0.523599 # -30 degrees in radians
    msg.angle_max = 3.66519 # 210 degrees in radians
    msg.angle_increment = 0.0698132 # 4 degrees in radians
    msg.ranges = ranges
    msg.scan_time = 0.1
    msg.time_increment = 0.1
    msg.range_min = 0.1
    msg.range_max = 3.0
    msg.intensities = []
    return msg
    
def dummy_pub():
    pub = rospy.Publisher("/laser",LaserScan, queue_size=10)
    rospy.init_node("Laser")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #rospy.loginfo("Publishing fake laser scan with %d points", len(produce().ranges))
        pub.publish(produce())
        rate.sleep()
        
if __name__ == '__main__':
    try:
        dummy_pub()
    except KeyboardInterrupt:
        print("Shutting Down")
