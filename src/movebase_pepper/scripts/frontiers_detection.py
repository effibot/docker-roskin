#!/home/dock/venvs/py27/bin/python
from copy import copy
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from movebase_pepper.msg import frontier_node, array_frontier_node
from sklearn.cluster import KMeans
class Frontier_Detection():
    def __init__(self):
        rospy.loginfo("Frontier Detection Node")
        self.frontiers = []
        self.map_topic = rospy.get_param("~map_topic")
        self.frontiers_pub = rospy.Publisher("/frontiers", array_frontier_node, queue_size=1)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.marker_pub = rospy.Publisher('frontier_markers_array', MarkerArray, queue_size=1)
        self.map = OccupancyGrid()
        self.map_data = []
        self.radius = rospy.get_param("~radius")    # m/pixel
        self.is_global = rospy.get_param("~global")
        if self.is_global==1:
            rospy.loginfo("Global Frontier Detection")
        elif self.is_global==0:
            rospy.loginfo("Local Frontier Detection")
        self.frontiers_list = []
        self.resolution = 0.05
        self.kmeans = KMeans(n_clusters=1, random_state=0)
        rospy.loginfo("Frontier Detection Node Initialized")

    # function to check adjacent frontiers
    def check_in_radius(self, frontier1, frontier2):
        dist = np.sqrt((frontier1[0]-frontier2[0])**2 + (frontier1[1]-frontier2[1])**2)
        return dist < self.radius

    def get_filtered_frontiers(self,points, k, radius):
        centroids = []
        centroid_number = np.min([k, len(points)])
        rospy.loginfo("Number of centroids: " + str(centroid_number))
        self.kmeans.n_clusters = centroid_number
        # from array of array to 2D array
        points_2d = np.array(points)
        # check if np.array is empty
        if points_2d.size != 0:
            self.kmeans.fit(points_2d)
            new_centroids = self.kmeans.cluster_centers_
            id = 0
            if self.frontiers_list == []:
                id = 0
            else:
                id = len(self.frontiers_list)
            # create frontier_node from centroids
            for i in range(len(new_centroids)):
                frontier = frontier_node()
                frontier.x = new_centroids[i][0]
                frontier.y = new_centroids[i][1]
                #frontier.radius = radius
                frontier.id = id
                # get from KMeans the list of points that belong to the cluster[i]
                frontier.blob = np.where(self.kmeans.labels_ == i)[0]
                # check in self.map_data if the centroid is free
                #if self.map_data[int(frontier.x/self.resolution) + int(frontier.y/self.resolution)*self.map.info.width] == 0:
                #    frontier.is_free = True
                ## set frontier visited to 0
                frontier.visit_counter = 0
                centroids.append(frontier)
            #rospy.loginfo(centroids)
        return centroids
    def update_frontiers(self, frontiers):
        self.frontiers_list = self.get_filtered_frontiers(frontiers, 10, self.radius)

    def getfrontier(self,msg):
        frontiers = []
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        data = msg.data
        unexplored = -1
        free_cell = 0
        if self.is_global == 0:
            unexplored = 0
        # Loop through the costmap data to find frontiers
        for i in range(height):
            for j in range(width):
                index = i * width + j
                if data[index] == unexplored:  # Unexplored region
                    # Check if the neighboring cells are free
                    is_frontier = False
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            nx = j + dx
                            ny = i + dy
                            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                                continue
                            neighbor_index = ny * width + nx
                            if data[neighbor_index] == 0 and unexplored==-1:  # Free cell
                                is_frontier = True
                                break
                            elif data[neighbor_index] ==0 and unexplored==0:  # Free cell
                                is_frontier = True
                                break
                        if is_frontier:
                            break

                    if is_frontier:
                        x = origin.x + (j + 0.5) * resolution
                        y = origin.y + (i + 0.5) * resolution
                        frontiers.append((x, y))  # (x, y) position
        rospy.loginfo("Frontiers found: " + str(len(frontiers)))
        return frontiers

    def publish_frontiers(self):
        # publish frontiers as MarkerArray each frontiers has a different color and all the frontier.points have the same color
        marker_array = MarkerArray()
        marker_array.markers = []
        i=0
        for fr in self.frontiers_list:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = fr.x
            marker.pose.position.y = fr.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            if self.is_global == 1:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif self.is_global == 0:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            i+=1
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


    def map_callback(self, msg):
        rospy.loginfo("Map received")
        self.map = msg
        self.map_data = msg.data
        self.resolution = msg.info.resolution

        self.update_frontiers(self.getfrontier(self.map))
        rospy.loginfo("Frontiers updated")
        rospy.loginfo("Number of frontiers: " + str(len(self.frontiers_list)))

    def run(self):
        while not rospy.is_shutdown():
            if self.frontiers_list != []:
                frontiers_array = array_frontier_node()
                frontiers_array.header.frame_id = "map"
                frontiers_array.header.stamp = rospy.Time.now()
                frontiers_array.frontier_list = self.frontiers_list
                self.frontiers_pub.publish(frontiers_array)
                self.publish_frontiers()
        pass
if __name__ == '__main__':
    rospy.init_node('frontiers')
    frontiers = Frontier_Detection()
    frontiers.run()
    rospy.spin()