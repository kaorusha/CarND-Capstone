#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32

import math

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        decel_limit = rospy.get_param('/dbw_node/decel_limit', -5.0)
        speed_limit = rospy.get_param('/waypoint_loader/velocity', 10)
        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2d = None
        self.pose = None
        self.kd_tree = None
        self.stop_wp_id = -1
        self.decel_limit = decel_limit
        self.speed_limit = speed_limit

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.kd_tree:
                id = self.get_closest_waypoint_id()
                self.publish_waypoints(id)
            rate.sleep()

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_waypoint_id = self.kd_tree.query([x, y], 1)[1]

        # check the closest waypoint is ahead of current pose
        closest_waypoint = self.waypoints_2d[closest_waypoint_id]
        prev_waypoint = self.waypoints_2d[closest_waypoint_id-1]
        waypoint_vector = np.array(closest_waypoint) - np.array(prev_waypoint)
        pose_vector = np.array([x, y]) - np.array(closest_waypoint)
        if np.dot(waypoint_vector, pose_vector) > 0:
            closest_waypoint_id = (
                closest_waypoint_id + 1) % len(self.waypoints_2d)
        return closest_waypoint_id

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx: closest_idx + LOOKAHEAD_WPS]
        if self.stop_wp_id != -1 and self.stop_wp_id < closest_idx + LOOKAHEAD_WPS:
            lane.waypoints = self.decelerate_waypoints(lane.waypoints, closest_idx)
        self.final_waypoints_pub.publish(lane)

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        stop_id = max(self.stop_wp_id - closest_idx - 5, 0)
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            dist = self.distance(waypoints, i, stop_id)
            vel = math.sqrt(2 * abs(self.decel_limit) * dist)
            vel = max(vel, 0.0)
            vel = min(self.speed_limit, vel)
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.base_waypoints = msg
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in msg.waypoints]
            self.kd_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_wp_id = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
