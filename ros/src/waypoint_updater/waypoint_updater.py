#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.register_subscribers()
        self.register_publishers()

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints= None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
    
    def register_publishers(self):
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
    def register_subscribers(self):
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)
      
    def loop(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                lane = self.generate_lane()
                self.final_waypoints_pub.publish(lane)
            r.sleep()
    
    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_waypoints.header
        
        closest_idx = self.get_closest_waypoint(self.pose)
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        
        waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]
        
        #no traffic lights close to car(ahead)
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(waypoints, closest_idx)
            
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp
        
    def get_closest_waypoint(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        
        closest_vect = np.array(self.waypoints_2d[closest_idx])
        prev_vect = np.array(self.waypoints_2d[closest_idx - 1])
        pose_vect = np.array([x,y])
        
        val = np.dot(closest_vect - prev_vect, pose_vect - closest_vect)
        if val > 0:
           closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
    
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #self.stopline_wp_idx = msg.data
        pass
        
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle = msg

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        updater = WaypointUpdater()
        updater.loop()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
