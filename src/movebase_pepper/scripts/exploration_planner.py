#!/home/dock/venvs/py27/bin/python

import rospy
from movebase_pepper.msg import array_frontier_node, frontier_node
from nav_msgs.msg import Odometry, OccupancyGrid
from message_filters import ApproximateTimeSynchronizer, Subscriber
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
import actionlib
from visualization_msgs.msg import Marker
import math
from geometry_msgs.msg import Quaternion
import random
class Exploration_planner():
    def __init__(self):
        rospy.loginfo("Exploration Planner Initialized")

        self.map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map_callback)
        self.global_map = None
        self.odom_topic = rospy.get_param('~odom_topic','/pepper_robot/odom')
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,self.planner)
        self.frontier_sub = rospy.Subscriber('/frontiers', array_frontier_node,self.frontiers_callback)
        self.goal = None
        self.marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.frontiers_list = []
    def frontiers_callback(self,data):
        self.frontiers_list = data
    def map_callback(self, msg):
        #rospy.loginfo("Map received")
        self.global_map = msg
    def nearest_frontier(self, robot_pose, frontiers):
        x_pos = robot_pose.position.x
        y_pos = robot_pose.position.y
        frontiers_pos = []
        for fr in frontiers.frontier_list:
            # retrive the cost of [fr.x,fr.y] in the global costmapfalse
            cost = self.global_map.data[int(fr.x/self.global_map.info.resolution) + int(fr.y/self.global_map.info.resolution)*self.global_map.info.width]
            if cost <0:
                frontiers_pos.append([fr.x, fr.y])
        # compute the distance between the robot and the frontiers in the map frame
        dist = [((x_pos - fr[0])**2 + (y_pos - fr[1])**2)**0.5 for fr in frontiers_pos]
        # get the first frontier in frontiers_pos with minimum distance and cost
        candidate = 0
        for i in range(len(dist)):
            # retrieve the cost of the frontier in the global map 2d array
            cost = self.global_map.data[int(frontiers_pos[i][0]) + int(frontiers_pos[i][1])*self.global_map.info.width]
            if dist[i] < dist[candidate] and dist[i] > 3:
                candidate = i
        # chose random index in len(dist)
        candidate = random.randint(0, len(dist))
        selected_pos = frontiers_pos[candidate]
        # compute tangent angle between the robot and the frontier
        orientation = [selected_pos[0] - x_pos, selected_pos[1] - y_pos]
        # compute angle between the robot and the frontier
        angle = math.atan2(orientation[1], orientation[0])
        # compute z and w of the quaternion
        z = math.sin(angle/2)
        w = math.cos(angle/2)
        return [selected_pos[0], selected_pos[1], z, w]
    def publish_marker(self, pos):
        to_pub = Marker()
        to_pub.header.frame_id = "map"
        to_pub.header.stamp = rospy.Time.now()
        to_pub.pose.position.x = pos[0]
        to_pub.pose.position.y = pos[1]
        to_pub.pose.position.z = 0
        to_pub.pose.orientation.x = 0.0
        to_pub.pose.orientation.y = 0.0
        to_pub.pose.orientation.z = 0.0
        to_pub.scale.x = 0.5
        to_pub.scale.y = 0.5
        to_pub.scale.z = 0.5
        to_pub.color.a = 0.0
        to_pub.color.r = 0.0
        to_pub.color.g = 1.0
        to_pub.type = Marker.SPHERE
        to_pub.id = 0
        to_pub.action = Marker.MODIFY
        self.marker_pub.publish(to_pub)
    def goal_done(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal Reached")
            rospy.loginfo("Canceling the goal")
            self.goal = None
            self.client.cancel_all_goals()
    def check_goal(self, data):
        # check if goal is near 10cm to inflation layer obstacles
        if self.goal is not None and self.global_map is not None:
            # retrieve all the costs around 5 pixel of the goal
            costs = []
            for i in range(-2, 2):
                for j in range(-2, 2):
                    costs.append(self.global_map.data[int(self.goal.target_pose.pose.position.x + i) + int(self.goal.target_pose.pose.position.y + j)*self.global_map.info.width])
            # check if a cost is greater than 1 (inflation layer) then return None
            for cost in costs:
                if cost > 1:
                    rospy.loginfo("Goal too close to an obstacle")
                    rospy.loginfo("Canceling the goal")
                    self.goal = None
                    self.client.cancel_all_goals()
                    return

    def planner(self, odom):
        robot_pose = odom.pose.pose
        # get the nearest frontier to the robot
        if self.global_map is not None and self.frontiers_list != []:
            # translate robot_pose (Point) from odom (Point) frame to map frame
            #robot_pose.position.x += self.global_map.info.origin.position.x
            #robot_pose.position.y += self.global_map.info.origin.position.y
            frontier = self.nearest_frontier(robot_pose, self.frontiers_list)
            if frontier is not None and self.goal is None:
                rospy.loginfo("New Goal Selected")
                rospy.loginfo("Sending the goal")
                self.client.wait_for_server()
                self.publish_marker(frontier)
                self.goal = MoveBaseGoal()
                self.goal.target_pose.header.frame_id = "map"
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = frontier[0]
                self.goal.target_pose.pose.position.y = frontier[1]
                self.goal.target_pose.pose.position.z = 0
                # assign orientation to the goal
                self.goal.target_pose.pose.orientation.x = 0.0
                self.goal.target_pose.pose.orientation.y = 0.0
                self.goal.target_pose.pose.orientation.z = frontier[2]
                self.goal.target_pose.pose.orientation.w = frontier[3]
                # send goal via actionlib
                self.client.send_goal(self.goal,done_cb = self.goal_done, feedback_cb = self.check_goal)
                self.client.wait_for_result()
    def run(self):
        pass


if __name__=="__main__":
    rospy.init_node('exploration_planner')
    exp_node = Exploration_planner()
    exp_node.run()
    rospy.spin()
