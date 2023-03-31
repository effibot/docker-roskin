
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10

# Initialize the BreezySLAM algorithm
slam = RMHC_SLAM(Laser(MAP_SIZE_PIXELS, MAP_SIZE_METERS),
                 MAP_SIZE_PIXELS, MAP_SIZE_METERS)

# Define the robot pose variables
x = 0.0
y = 0.0
theta = 0.0
last_odom_time = None

# Define the callback function for the laser scans


def lidar_callback(scan):
    global x, y, theta, last_odom_time

    # Convert the LaserScan message to a list of distances in millimeters
    scans_mm = [int(x*1000) for x in scan.ranges]

    # Get the time difference since the last odometry update
    dt = rospy.Time.now() - last_odom_time

    # Calculate the pose change from the odometry data
    dxy_mm = 0.0
    dtheta_degrees = 0.0
    if dt > rospy.Duration(0):
        dxy_mm = odom.twist.twist.linear.x * dt.to_sec() * 1000.0
        dtheta_degrees = odom.twist.twist.angular.z * dt.to_sec() * 180.0 / math.pi
    pose_change = (dxy_mm, dtheta_degrees, dt.to_sec())

    # Update the BreezySLAM algorithm with the latest scan and odometry data
    slam.update(scans_mm, pose_change=pose_change)

    # Get the estimated robot pose and map
    pose = slam.getpos()
    mapbytes = slam.getmap()

    # Publish the robot pose and map
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'map'
    pose_msg.pose.position.x = pose[0]/1000.0
    pose_msg.pose.position.y = pose[1]/1000.0
    pose_msg.pose.position.z = 0
    q = Quaternion()
    q.x, q.y, q.z, q.w = 0, 0, 0, 1
    pose_msg.pose.orientation = q
    pose_pub.publish(pose_msg)

    map_pub.publish(mapbytes)

    # Update the last odometry time
    last_odom_time = rospy.Time.now()

# Define the callback function for the odometry data


def odom_callback(odom):
    global x, y, theta, last_odom_time
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    q = odom.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    theta = math.degrees(theta)
    last_odom_time = odom.header.stamp


if __name__ == '__main__':
    rospy.init_node('breezyslam_node')
    lidar = Lidar()
    lidar.connect


# LASER CALLBACK
    print(str(data))
    msg = yaml.safe_load(str(data))
    # Get the byte offsets of the x, y, and z fields
    x_offset = msg['fields'][0]['offset']
    y_offset = msg['fields'][1]['offset']
    z_offset = msg['fields'][2]['offset']

    # Get the data type of the x, y, and z fields (7 corresponds to 'float32')
    data_type = 'f'
    x_dtype = msg['fields'][0]['datatype']
     y_dtype = msg['fields'][1]['datatype']
      z_dtype = msg['fields'][2]['datatype']
       if x_dtype == 7:
            data_type = '<f'
        elif x_dtype == 8:
            data_type = '<d'

        # Extract the x, y, and z values from the data field
        data = msg['data']
        x_bytes = data[x_offset:x_offset+4]
        y_bytes = data[y_offset:y_offset+4]
        z_bytes = data[z_offset:z_offset+4]
        x = struct.unpack(data_type, bytes(bytearray(x_bytes)))[0]
        y = struct.unpack(data_type, bytes(bytearray(y_bytes)))[0]
        z = struct.unpack(data_type, bytes(bytearray(z_bytes)))[0]

        print("x:", x)
        print("y:", y)
        print("z:", z)


# subscribe to three topic (left,front,right) and then store thos values in a array in every cycle then publish it to a topic called /pepper/scan

def run(self):
        rospy.init_node('pepper_lidar_node')
        rospy.Subscriber('/pepper/scan_left',
                         LaserScan, self.scan_callback)
        rospy.Subscriber('/pepper/scan_front',
                         LaserScan, self.scan_callback)
        rospy.Subscriber('/pepper/scan_right',
                         LaserScan, self.scan_callback)
        rospy.spin()

def scan_callback(self, data):
        self.scan_data.append(data)
        if len(self.scan_data) == 3:
            self.publish_scan()
            self.scan_data = []
