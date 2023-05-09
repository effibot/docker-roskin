#!/home/dock/venvs/py27/bin/python

#    /naoqi_driver/laser publishes a laserscan message with 60 ranges, 0,0698132 (4 degrees) radians between each range.
#    The first range is -0,523599 radians (-30 degrees), the last is at 3,83972 radians (220 degrees).
#    Every 15 ranges a gap of eight '-1' values are inserted to simulate the void between the three laserscanners.
#    The first 15 ranges are the right laserscanner, the next 15 are the front laserscanner and the last 15 are the left laserscanner.
#    Ranges are in meters.

#[0.45477495 0.73922729 0.74112463 0.63749236 0.60998499 0.61063629
 #0.67930669 1.22988236 0.78747702 0.84909689 0.82142103 1.32026827
 #1.3532896  1.35054481 1.38477623]

#[0.45477494597435, 0.739227294921875, 0.7411246299743652, 0.6374923586845398, 0.6099849939346313, 0.610636293888092, 0.67930668592453, 1.2298823595046997, 0.7874770164489746, 0.8490968942642212, 0.8214210271835327, 1.3202682733535767, 1.3532896041870117, 1.350544810295105, 1.38477623462677, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 7.050281047821045, 7.051639080047607, 7.052820205688477, 7.053819179534912, 1.3611520528793335, 1.3617579936981201, 3.186048746109009, 7.055942058563232, 1.4020845890045166, 7.055856704711914, 7.055526256561279, 7.055004596710205, 0.6873072981834412, 0.6963121294975281, 0.7158398032188416, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.5097943544387817, 1.5606811046600342, 0.6713876128196716, 0.5920705199241638, 0.5750609040260315, 0.5827099084854126, 1.797183871269226, 1.7979685068130493, 1.7983046770095825, 7.091649055480957, 0.7240342497825623, 0.723102867603302, 0.44776394963264465, 0.438874751329422, 0.47583523392677307]
#
#
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class laser_listener():

    def __init__(self, factor = 2, rate = 10):
        self.angle_min = -0.523599 # -30 degrees in radians
        self.right_angle_max = 0.523599 # 30 degrees in radians
        self.center_angle_min = 1.0472 # 60 degrees in radians
        self.center_angle_max = 2.0944 # 120 degrees in radians
        self.left_angle_min = 2,61799 # 150 degrees in radians
        self.angle_max = 3,66519 # 210 degrees in radians
        self.angle_increment = 0.0698132 # 4 degrees in radians
        self.factor = factor
        rospy.Subscriber("/naoqi_driver/laser",LaserScan, self.listen_callback)
        pub = rospy.Publisher("/laser",LaserScan, queue_size=10)
        rospy.init_node("Laser")
        self.rate = rospy.Rate(rate)

    def listen_callback(self,msg):
        # get the ranges from the laserscanner
        scan_data = np.array(msg.ranges)
        right_scan = scan_data[0:15]
        rospy.loginfo("Right Scan: (%d): %s", len(right_scan), right_scan)
        right_void = scan_data[16:23]
        rospy.loginfo("Right Void (%d): %s", len(right_void), right_void)
        center_scan = scan_data[24:38]
        rospy.loginfo("Center Scan: %s", center_scan)
        left_void = scan_data[39:46] # useless
        rospy.loginfo("Left Void (%d): %s", len(left_void), left_void)
        left_scan = scan_data[47:63]
        rospy.loginfo("Left Scan: (%d): %s", len(left_scan), left_scan)
        
        # expand the scan to get a higher resolution
        expanded_right_scan = self.expand_scan(right_scan, self.factor, self.angle_min, self.right_angle_max)
        #rospy.loginfo("Expanded Right Scan (%d): %s\n", len(expanded_right_scan), expanded_right_scan)
        expanded_rigth_void = -1*np.ones(self.factor*len(right_void)-1)
        expanded_center_scan = self.expand_scan(center_scan, self.factor, self.center_angle_min, self.center_angle_max)
        expanded_left_void = expanded_rigth_void
        expanded_left_scan = self.expand_scan(left_scan, self.factor, self.left_angle_min, self.angle_max)
        #
        # concatenate the expanded scans
        #expanded_scan = np.concatenate((expanded_right_scan, expanded_rigth_void, expanded_center_scan, expanded_left_void, expanded_left_scan))
        
        
    def expand_scan(self, scan, factor, angle_start, angle_end):
        scan_lenght = len(scan) # 15
        expanded_scan = np.zeros(factor*scan_lenght-1) # default value is 150
        # pre-fill the expanded scan with the true scan points
        expanded_scan[0] = scan[0]
        for i in range(1, scan_lenght):
            # get previous and current scan point x,y coordinates
            pred = [scan[i-1]*np.cos(angle_start),scan[i-1]*np.sin(angle_start)]
            curr = [scan[i]*np.cos(angle_end),scan[i]*np.sin(angle_end)]
            # get the midpoint between the previous and current scan point
            mid = [(pred[0]+curr[0])/2, (pred[1]+curr[1])/2]
            # pre-fill the expanded scan with the true scan points
            expanded_scan[i*factor] = scan[i]
            # now expand
            for j in range(1,factor):
                # fill the expanded scan with the expanded scan points
                expanded_scan[(i-1)*factor+j] = np.sqrt(mid[0]**2+mid[1]**2)
        return expanded_scan

if __name__ =='__main__':
    listener = laser_listener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
