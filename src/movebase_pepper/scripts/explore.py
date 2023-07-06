#!/home/dock/venvs/py27/bin/python

import rospy
import qi
from naoqi_driver.naoqi_node import NaoqiNode
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry, Path
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
import random
#from movebase_pepper.scripts.exploration_graph import Graph, Node
class Explore(NaoqiNode):
    def __init__(self):
        self.global_map = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_map_sub)
        self.odom = rospy.Subscriber("/pepper/odom", Odometry, self.exploration_sub)
        self.global_map_obj = None
        self.frontiers = []
        self.marker_pub = rospy.Publisher('frontier_markers_array', MarkerArray, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.current_goal = []
        self.lost_goal = []

    def is_frontier(self,data, width, height, x, y):
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                nx = x + dx
                ny = y + dy
                if nx >= 0 and nx < width and ny >= 0 and ny < height:
                    index = ny * width + nx
                    if data[index] == 0:
                        return True
        return False

    def add_frontiers(self,global_costmap_message, min_size):
        width = global_costmap_message.info.width
        height = global_costmap_message.info.height
        resolution = global_costmap_message.info.resolution
        data = global_costmap_message.data
        frontiers = []
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if data[index] == -1:
                    if self.is_frontier(data, width, height, x, y):
                        frontier = (x * resolution, y * resolution)
                        if frontier not in frontiers:
                            # transform the frontier to odom frame
                            frontier_list = list(frontier)
                            frontier_list[0] = frontier_list[0] + global_costmap_message.info.origin.position.x
                            frontier_list[1] = frontier_list[1] + global_costmap_message.info.origin.position.y
                            frontiers.append(tuple(frontier_list))
        return frontiers
    def publish_Marker(self, frontiers,red=1.0,green=0.0,blue=0.0,scale_x=0.05,scale_y=0.05):
        marker_array_msg = MarkerArray()
        marker_array_msg.markers = []
        i=0
        for frontier in frontiers:
            marker_msg = Marker()
            marker_msg.header.frame_id = "map"
            marker_msg.header.stamp = rospy.Time.now()
            marker_msg.ns = "frontiers"
            marker_msg.id = i
            marker_msg.type = Marker.SPHERE
            marker_msg.action = 0
            marker_msg.pose.orientation.w = 1.0
            marker_msg.scale.x = scale_x
            marker_msg.scale.y = scale_y
            marker_msg.scale.z = 0.05
            marker_msg.color.a = 1.0
            marker_msg.color.r = red
            marker_msg.color.g = green
            marker_msg.color.b = blue
            marker_msg.pose.position.x = frontier[0]
            marker_msg.pose.position.y = frontier[1]
            marker_msg.pose.position.z = 0.0
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            if self.current_goal is not None:
                if frontier[0] == self.current_goal[0] and frontier[1] == self.current_goal[1]:
                    marker_msg.color.r = 0.0
                    marker_msg.color.g = 1.0
                    marker_msg.color.b = 0.0
                    marker_msg.scale.x = 0.2
                    marker_msg.scale.y = 0.2
                    marker_msg.scale.z = 0.2
            marker_array_msg.markers.append(marker_msg)
            i+=1
        self.marker_pub.publish(marker_array_msg)

    def update_frontiers(self, global_costmap_message):
        width = global_costmap_message.info.width
        height = global_costmap_message.info.height
        resolution = global_costmap_message.info.resolution
        data = global_costmap_message.data
        updated_frontiers = []
        rospy.loginfo("Update the frontiers")
        for frontier in self.frontiers:
            x = int(frontier[0] / resolution)
            y = int(frontier[1] / resolution)
            index = y * width + x

            if index >= 0 and index < len(data) and data[index] == -1 and self.is_frontier(data, width, height, x, y):
                updated_frontiers.append(frontier)
        return updated_frontiers

    def global_map_sub(self,costmap):
        self.global_map_obj = costmap
        rospy.loginfo("Detecting Frontiers")
        self.frontiers = self.update_frontiers(self.global_map_obj)
        frontier_list = self.add_frontiers(self.global_map_obj,min_size=4)
        self.frontiers.extend(frontier_list)
        self.frontiers  = list(set(self.frontiers))
        rospy.loginfo("Number of frontiers: %d", len(self.frontiers))
        self.publish_Marker(self.frontiers,1.0,0.0,0.0,0.05,0.05)
    def move_base_status(self,goal):
        status = self.client.get_state()
        if status == actionlib.GoalStatus.PENDING:
            rospy.loginfo("Goal status: PENDING")
        elif status == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Goal status: ACTIVE")
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Goal status: PREEMPTED")
        elif status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal status: SUCCEEDED")
        elif status == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Goal status: ABORTED")
        elif status == actionlib.GoalStatus.REJECTED:
            rospy.loginfo("Goal status: REJECTED")
        elif status == actionlib.GoalStatus.RECALLED:
            rospy.loginfo("Goal status: RECALLED")
        elif status == actionlib.GoalStatus.LOST:
            rospy.loginfo("Goal status: LOST")
            pos = tuple([goal.target_pose.pose.position.x,goal.target_pose.pose.position.y])
            if pos not in self.lost_goal:
                self.lost_goal.append(pos)

    def select_goal(self, robotpose, frontiers):
        if len(self.lost_goal)>0:
            rospy.loginfo("Lost goal %d",len(self.lost_goal))
            rospy.loginfo("Frontiers %d",len(frontiers))
            frontiers = [x for x in frontiers if x not in self.lost_goal]
        rospy.loginfo("Frontiers filtered to %d",len(frontiers))
        calculated_distance = []
        # calculate euclidean distance from robot pose to each frontier
        rospy.loginfo("Selecting goal")
        for frontier in frontiers:
            distance = np.linalg.norm(np.array(robotpose) - np.array(frontier))
            calculated_distance.append(distance)
        # Retrieve indexes of the maximum distance        rospy.loginfo("Goal selected")
        max_distance_indexes = np.argwhere(calculated_distance > (np.amax(calculated_distance)-0.5))
        # Select the farthest frontier        rospy.loginfo("Retrieving the max")
        pos = None
        rospy.loginfo("Number of candidates %d",len(max_distance_indexes))
        while pos is None:
            pos = frontiers[max_distance_indexes[0][0]]
            rospy.loginfo("Goal selected")
            angle = math.atan2(pos[1] - robotpose[1], pos[0] - robotpose[0])
            z = math.sin(angle / 2.0)
            w = math.cos(angle / 2.0)
            return [pos[0], pos[1], z,w]
        return [self.current_goal[0], self.current_goal[1], self.current_goal[2], self.current_goal[3]]
    def on_done(self,state,data):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
        else:
            rospy.loginfo("Goal failed with state: %s", state)

    def on_active(self):
        rospy.loginfo("On Active")
    def on_feedback(self,feedback):
        rospy.loginfo("On Feedback")
        rospy.loginfo(feedback)


    def sendgoal(self,goal_target):
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = goal_target[0]
        goal.target_pose.pose.position.y = goal_target[1]
        goal.target_pose.pose.orientation.z = goal_target[2]
        goal.target_pose.pose.orientation.w = goal_target[3]
        rospy.loginfo("Sending the goal")
        self.client.send_goal(goal,done_cb=self.on_done,active_cb=self.on_active,feedback_cb=self.on_feedback)
        self.client.wait_for_result(rospy.Duration(1))
        # Check if the goal was successful
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("Goal reached successfully!")
        else:
            # Check if the goal failed due to "Failed to get a path"
            if self.client.get_state() == actionlib.GoalStatus.ABORTED:
                result_text = self.client.get_goal_status_text()
                if "Failed to get a path" in result_text:
                    print("Goal aborted due to failure in path planning.")
                else:
                    print("Goal aborted for a different reason.")
            else:
                print("Goal failed for a different reason.")


    def exploration_sub(self,odom):
        if self.global_map_obj is not None:
            robotpose = [odom.pose.pose.position.x, odom.pose.pose.position.y]
            # select the farthers frontiers from robot pose ins self.frontiers
            goal = self.select_goal(robotpose, self.frontiers)
            #if self.current_goal is not None:
            rospy.loginfo("Goal: %f, %f,%f", goal[0], goal[1],goal[2])
            # create a goal message to MoveBaseAction
            self.current_goal = goal
            self.sendgoal(self.current_goal)

if __name__ == '__main__':
    rospy.loginfo("Exploration Node")
    rospy.init_node('exploration_node')
    exploration_node=Explore()
    rospy.spin()
    exit(0)
