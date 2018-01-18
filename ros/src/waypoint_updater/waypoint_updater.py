#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import copy
import tf
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
TARGET_SPEED_MPH = 10 # miles per hour
TARGET_SPEED_KPH = TARGET_SPEED_MPH*1.609344 # km per hour
TARGET_SPEED_MPS = 1000*TARGET_SPEED_KPH/3600 # meters per second


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_waypoints = None
        self.last_waypoint_index = None
        self.num_waypoints = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        self.base_waypoints_subscriber = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg
        # rospy.loginfo("Current position (%.2f, %.2f)" % (msg.pose.position.x, msg.pose.position.y))
        if self.base_waypoints:
            self.last_waypoint_index = self.find_next_closest_waypoint()

            if self.last_waypoint_index + LOOKAHEAD_WPS > self.num_waypoints:
                next_waypoints = self.base_waypoints[self.last_waypoint_index:] + \
                    self.base_waypoints[0:LOOKAHEAD_WPS-(self.num_waypoints-self.last_waypoint_index)]
            else:
                next_waypoints = self.base_waypoints[self.last_waypoint_index:self.last_waypoint_index+LOOKAHEAD_WPS]

            for i in range(len(next_waypoints)):
                self.set_waypoint_velocity(next_waypoints, i, TARGET_SPEED_MPS)

            waypoint_x = self.base_waypoints[self.last_waypoint_index].pose.pose.position.x
            waypoint_y = self.base_waypoints[self.last_waypoint_index].pose.pose.position.y

            next_waypoints_msg = Lane()
            next_waypoints_msg.header.frame_id = '/world'
            next_waypoints_msg.header.stamp = rospy.Time(0)
            next_waypoints_msg.waypoints = next_waypoints

            rospy.loginfo("Next waypoint %d" % self.last_waypoint_index)
            rospy.loginfo("Next waypoint position (%.2f, %.2f)" % (waypoint_x, waypoint_y))

            self.final_waypoints_pub.publish(next_waypoints_msg)

    def waypoints_cb(self, waypoints):
        rospy.logwarn("Message received on /base_waypoints")
        self.base_waypoints = waypoints.waypoints
        self.num_waypoints = len(self.base_waypoints)
        self.base_waypoints_subscriber.unregister()
        rospy.logwarn("Num waypoints: %d" % self.num_waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def find_next_closest_waypoint(self):
        SEARCH_AHEAD = 120
        next_index = None
        magnitude = lambda a: a.pose.pose.position.x**2 + a.pose.pose.position.y**2
        if self.last_waypoint_index == None:
            distance = 10000
            transformed_waypoints = self.get_transformed_waypoints(self.current_pose, self.base_waypoints)
            for i in range(self.num_waypoints):
                wp_dist = magnitude(transformed_waypoints[i])
                if transformed_waypoints[i].pose.pose.position.x > 0 and wp_dist < distance:
                    next_index = i
                    distance = wp_dist
            rospy.loginfo("Minimum distance on first iteration %.3f" % distance)
        elif self.last_waypoint_index + SEARCH_AHEAD > self.num_waypoints:
            transformed_waypoints = self.get_transformed_waypoints(self.current_pose, self.base_waypoints[self.last_waypoint_index:])
            for i in range(self.num_waypoints - self.last_waypoint_index):
                if transformed_waypoints[i].pose.pose.position.x > 0:
                    next_index = self.last_waypoint_index + i
                    break
            if not self.last_waypoint_index:
                transformed_waypoints = self.get_transformed_waypoints(self.current_pose,
                                                                       self.base_waypoints[:SEARCH_AHEAD])
                for i in range(SEARCH_AHEAD):
                    if transformed_waypoints[i].pose.pose.position.x > 0:
                        next_index = i
                        break
        else:
            transformed_waypoints = self.get_transformed_waypoints(self.current_pose,
                                                                   self.base_waypoints[self.last_waypoint_index:self.last_waypoint_index + SEARCH_AHEAD])
            for i in range(SEARCH_AHEAD):
                if transformed_waypoints[i].pose.pose.position.x > 0:
                    next_index = self.last_waypoint_index + i
                    break
        if not next_index:
            rospy.logfatal("Next index not found!!! Requires urgent attention")
        return next_index

    def get_transformed_waypoints(self, current_pose, waypoints):
        waypoints = copy.deepcopy(waypoints)
        x_pos = current_pose.pose.position.x
        y_pos = current_pose.pose.position.y
        quaternion = (current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                      current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        yaw = euler[2]

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        for waypoint in waypoints:
            shift_x = waypoint.pose.pose.position.x - x_pos
            shift_y = waypoint.pose.pose.position.y - y_pos

            waypoint.pose.pose.position.x = shift_x * cos_yaw + shift_y * sin_yaw
            waypoint.pose.pose.position.y = shift_y * cos_yaw - shift_x * sin_yaw

        return waypoints



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')