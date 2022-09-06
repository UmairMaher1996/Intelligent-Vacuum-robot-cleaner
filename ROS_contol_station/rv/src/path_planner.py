#!/usr/bin/env python

import roslib;
import rospy
import tf
from tf.transformations import quaternion_from_euler as qfe
from actionlib import SimpleActionClient

import numpy as np
from math import radians

from geometry_msgs.msg import PolygonStamped, Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

import shapely.geometry as geo

class PathPlannerNode(object):
    
    def __init__(self):
        # Setup ROS node
        rospy.init_node('path_planner')

        # ROS params
        self.cut_spacing = rospy.get_param("~coverage_spacing", 0.2)

        # Setup publishers and subscribers
        rospy.Subscriber('rv_area', PolygonStamped, self.field_callback)
        self.path_marker_pub = rospy.Publisher('visualization_marker',
                                               MarkerArray,
                                               latch=True)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Setup initial variables
        self.field_shape = None
        self.field_frame_id = None
        self.path = None
        self.path_status = None
        self.path_markers = None
        self.start_path_following = False
        self.robot_pose = None
        self.goal_state = None
        self.current_destination = None
        self.testing = False
        self.current_distance = None
        self.previous_destination = None
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        self.just_reset = False
        self.timeout = False

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.start_path_following:
                heading = 0
                self.setup_path_following(heading)    
                while not rospy.is_shutdown():
                    if not self.step_path_following():
                        break
                self.start_path_following = False



    def field_callback(self, msg):
        
        temp_points = []
        for point in msg.polygon.points:
            temp_points.append( (float(point.x), float(point.y)) )
        self.field_shape = geo.Polygon(temp_points)
        self.field_frame_id = msg.header.frame_id
        self.start_path_following = True

    def odom_callback(self, msg):
        
        self.robot_pose = msg

    def plan_path(self, field_polygon, origin=None, degrees=0):
       
        
        from automow_planning.maptools import rotation_tf_from_longest_edge, RotationTransform
        rotation = rotation_tf_from_longest_edge(field_polygon)
        rotation = RotationTransform(rotation.w + degrees)
        
        from automow_planning.maptools import rotate_polygon_to
        transformed_field_polygon = rotate_polygon_to(field_polygon, rotation)
        
        from automow_planning.coverage import decompose
        print origin
        if origin is not None:
            point_mat = np.mat([[origin[0], origin[1], 0]], dtype='float64').transpose()
            origin = rotation.irm * point_mat
            origin = (origin[0,0], origin[1,0])
        transformed_path = decompose(transformed_field_polygon,
                                     origin=(origin[0], origin[1]),
                                     width=self.cut_spacing)
        
        from automow_planning.maptools import rotate_from
        self.path = rotate_from(np.array(transformed_path), rotation)
        
        self.path = self.calculate_headings(self.path)
        
        self.path_status = []
        for waypoint in self.path:
            self.path_status.append('not_visited')
        
        self.visualize_path(self.path, self.path_status)

    def calculate_headings(self, path):
        
        new_path = []
        for index, waypoint in enumerate(path):
            new_path.append(list(path[index]))
            
            if index == 0:
                new_path[index].append(0)
                continue
            
            dx = path[index][0] - path[index-1][0]
            dy = path[index][1] - path[index-1][1]
            from math import atan2, pi
            heading = atan2(dy, dx)
            new_path[index].append(heading)
        return new_path

    def visualize_path(self, path, path_status=None):
        
        self.visualize_path_as_marker(path, path_status)

    def visualize_path_as_path(self, path, path_status=None):
        
        now = rospy.Time.now()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = self.field_frame_id
        for index, waypoint in enumerate(path):
            if path_status != None:
                if path_status[index] == 'visited': 
                    try:
                        if path_status[index+1] == 'visited':   
                            continue
                    except KeyError as e: 
                        pass
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.field_frame_id
            pose_msg.pose.position.x = waypoint[0]
            pose_msg.pose.position.y = waypoint[1]
            msg.poses.append(pose_msg)

    def visualize_path_as_marker(self, path, path_status):
        
        now = rospy.Time.now()
        if self.path_markers == None:
            self.path_markers = MarkerArray()
        self.path_markers = MarkerArray()
        line_strip_points = []
        for index, waypoint in enumerate(path):
            waypoint_marker = Marker()
            waypoint_marker.header.stamp = now
            waypoint_marker.header.frame_id = self.field_frame_id
            waypoint_marker.ns = "waypoints"
            waypoint_marker.id = index
            waypoint_marker.type = Marker.ARROW
            if index == 0:
                waypoint_marker.type = Marker.CUBE
            waypoint_marker.action = Marker.MODIFY
            waypoint_marker.scale.x = 1
            waypoint_marker.scale.y = 1
            waypoint_marker.scale.z = 0.25
            point = Point(waypoint[0], waypoint[1], 0)
            waypoint_marker.pose.position = point

            line_strip_points.append(point)
 
            quat = qfe(0, 0, waypoint[2])
            waypoint_marker.pose.orientation.x = quat[0]
            waypoint_marker.pose.orientation.y = quat[1]
            waypoint_marker.pose.orientation.z = quat[2]
            waypoint_marker.pose.orientation.w = quat[3]

            status = path_status[index]
            if status == 'not_visited':
                waypoint_marker.color = ColorRGBA(1,0,0,0.5)
            elif status == 'visiting':
                waypoint_marker.color = ColorRGBA(0,1,0,0.5)
            elif status == 'visited':
                waypoint_marker.color = ColorRGBA(0,0,1,0.5)
            else:
                rospy.err("Invalid path status.")
                waypoint_marker.color = ColorRGBA(1,1,1,0.5)

            self.path_markers.markers.append(waypoint_marker)

        line_strip = Marker()
        line_strip.header.stamp = now
        line_strip.header.frame_id = self.field_frame_id
        line_strip.ns = "lines"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.1
        line_strip.color = ColorRGBA(0,0,1,0.5)
        line_strip.points = line_strip_points
        self.path_markers.markers.append(line_strip)
        self.path_marker_pub.publish(self.path_markers)

    def setup_path_following(self, degrees=0):
        
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        connected_to_move_base = False
        dur = rospy.Duration(1.0)
        if self.testing:
            self.robot_pose = Odometry()
            self.robot_pose.pose.pose.position.x = 0
            self.robot_pose.pose.pose.position.y = 0
        while self.field_shape == None:

            if rospy.is_shutdown(): return
            msg = "Qualification: waiting on the field shape."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()

        while self.robot_pose == None:
            if rospy.is_shutdown(): return
            msg = "Qualification: waiting on initial robot pose."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        origin = (self.robot_pose.pose.pose.position.x,
                  self.robot_pose.pose.pose.position.y)
        self.plan_path(self.field_shape, origin, degrees)
        rospy.loginfo("Path Planner: path planning complete.")
        while not connected_to_move_base:
            connected_to_move_base = self.move_base_client.wait_for_server(dur)
            if rospy.is_shutdown(): return
            msg = "Path Planner: waiting on move_base."
            rospy.loginfo(msg)
        return

    def get_next_waypoint_index(self):
        
        for index, status in enumerate(self.path_status):
            if status == 'visited':
                continue
            if status == 'visiting':
                return index
            if status == 'not_visited':
                return index
        return None

    def distance(self, p1, p2):
        
        from math import sqrt
        dx = p2.target_pose.pose.position.x - p1.target_pose.pose.position.x
        dy = p2.target_pose.pose.position.y - p1.target_pose.pose.position.y
        return sqrt(dx**2 + dy**2)

    def step_path_following(self):
        
        self.visualize_path(self.path, self.path_status)
        current_waypoint_index = self.get_next_waypoint_index()
        if current_waypoint_index == None:
            rospy.loginfo("Path Planner: Done.")
            return False
        if current_waypoint_index == 0:
            self.path_status[current_waypoint_index] = 'visited'
        current_waypoint = self.path[current_waypoint_index]
        current_waypoint_status = self.path_status[current_waypoint_index]
        if current_waypoint_status == 'visited':
            return True
        if current_waypoint_status == 'not_visited':
            self.path_status[current_waypoint_index] = 'visiting'
            destination = MoveBaseGoal()
            destination.target_pose.header.frame_id = self.field_frame_id
            destination.target_pose.header.stamp = rospy.Time.now()
  
            destination.target_pose.pose.position.x = current_waypoint[0]
            destination.target_pose.pose.position.y = current_waypoint[1]

            if self.previous_destination == None:
                self.current_distance = 5.0
            else:
                self.current_distance = self.distance(self.previous_destination, destination)
            quat = qfe(0, 0, current_waypoint[2])
            destination.target_pose.pose.orientation.x = quat[0]
            destination.target_pose.pose.orientation.y = quat[1]
            destination.target_pose.pose.orientation.z = quat[2]
            destination.target_pose.pose.orientation.w = quat[3]
            rospy.loginfo("Sending waypoint (%f, %f)@%f" % tuple(current_waypoint))
            self.current_destination = destination
            self.move_base_client.send_goal(destination)
            self.previous_destination = destination
        temp_state = self.move_base_client.get_goal_status_text()
        if current_waypoint_status == 'visiting':
            if temp_state in ['ABORTED', 'SUCCEEDED']:
                self.path_status[current_waypoint_index] = 'visited'
            else:
                duration = rospy.Duration(2.0)
                from math import floor
                count = 0
                self.move_base_client.wait_for_result()
                if self.timeout == True:
                    if count == 6+int(self.current_distance*4):
                        rospy.logwarn("Path Planner: move_base goal timeout occurred, clearing costmaps")
                        self.move_base_client.cancel_all_goals()
                        self.clear_costmaps()
                        rospy.Rate(1.0).sleep()
                        if not self.just_reset:
                            self.just_reset = True
                            self.path_status[current_waypoint_index] = 'not_visited'
                        else:
                            self.just_reset = False
                            self.path_status[current_waypoint_index] = 'visited'
                        return True
                self.path_status[current_waypoint_index] = 'visited'
        return True

if __name__ == '__main__':
    ppn = PathPlannerNode()
