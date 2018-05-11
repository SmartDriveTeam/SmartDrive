#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd
from std_msgs.msg import Bool

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
RED_LIGHT_LOOKAHEAD_WPS = 30

LOGWARN_MODE = False ## Show logwarn if True is set
TO_METER_PER_SEC = 0.44704 # 1 mile/hour = 0.44704 meter/sec
TARGET_VEL_MPH = 20
TARGET_VEL = TARGET_VEL_MPH * TO_METER_PER_SEC

MIN_TRAFFIC_LIGHT_DIST = 10  # min distance from traffic light when seeing red light  
MAX_TRAFFIC_LIGHT_DIST = 40

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        sub3 = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.throttle_cmd_cb)
        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steering_cmd_cb)
        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cmd_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None  # receive pose msg
        self.twist = None  # receive velocity msg
        self.current_vel = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.closest_waypoint_idx = None
 
        self.red_light_waypoints = None
        self.red_light_waypoint_idx = None

        self.throttle_cmd = None
        self.steering_cmd = None
        self.brake_cmd = None

        self.update_waypoints()

    def update_waypoints(self): ##
        rate = rospy.Rate(10)	# decrease Rate() from 50 to 10 Hz for slow VM & Host interaction
        while not rospy.is_shutdown():
            if self.pose and self.twist and self.base_waypoints:
                self.closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.current_vel = self.twist.twist.linear.x
                if self.red_light_waypoint_idx:  # recognize traffic light
                    if self.red_light_waypoint_idx == -1:
                        # 1. Green or Yellow Light
                        self.publish_waypoints(self.closest_waypoint_idx)
                        if LOGWARN_MODE:
                            rospy.logwarn("[TL Green or Yellow] VEL = %s", self.current_vel)

                    else:  
                        # 2. Red Light
                        # Estimate distance from stop line
                        self.red_light_waypoints = self.base_waypoints.waypoints[self.closest_waypoint_idx:self.red_light_waypoint_idx]

                        len_wps = len(self.red_light_waypoints)
                        tl_dist = self.distance(self.red_light_waypoints, 0, len_wps-1)
                   
                        self.publish_red_light_waypoints(self.closest_waypoint_idx, self.red_light_waypoint_idx)

                        if LOGWARN_MODE:
                            rospy.logwarn("[TL RED] TL_DIST = %s, VEL = %s",tl_dist, self.current_vel)

                #rospy.logwarn("next_idx, red_idx = %s, %s", self.closest_waypoint_idx, self.red_light_waypoint_idx) 
       
                # Status              
                #if LOGWARN_MODE:
                #    rospy.logwarn("x, y, next_idx, red_idx = %s, %s, %s, %s",self.pose.pose.position.x, self.pose.pose.position.y, self.closest_waypoint_idx, self.red_light_waypoint_idx)    # red_idx -1 : Y,G    not -1 : red light waypoint index
                #    rospy.logwarn("throttle, sttering, brake = %s, %s, %s",self.throttle_cmd, self.steering_cmd, self.brake_cmd)
                
            rate.sleep()


    def get_closest_waypoint_idx(self): ##
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        #Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx): ##
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        for i in range(len(lane.waypoints)-1):
	    self.set_waypoint_velocity(lane.waypoints, i, TARGET_VEL)
        self.final_waypoints_pub.publish(lane)

    def publish_red_light_waypoints(self, closest_idx, red_light_waypoint_idx): ##
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx: red_light_waypoint_idx + RED_LIGHT_LOOKAHEAD_WPS]
        len_tl_wps = red_light_waypoint_idx - closest_idx 
        len_wps = len(lane.waypoints)  # including LOOK
        
        d = self.distance(lane.waypoints, 0, len_tl_wps-1)
        #rospy.logwarn("%s, %s, %s", closest_idx, red_light_waypoint_idx, d)
        
        for idx in range(len_tl_wps):
            if (d > MIN_TRAFFIC_LIGHT_DIST and d < MAX_TRAFFIC_LIGHT_DIST): # change speed within 10~40 mile before red light
                if self.current_vel < 1 : # Too slow, keep constant low speed approaching red light
                    v = 1
                else:  # Too fast, slow down gradually
                    v = self.current_vel * (1 - float(idx)/(len_wps-1))
            else:  
                v = 0
            self.set_waypoint_velocity(lane.waypoints, idx, v)
            
        for idx in range(len_tl_wps, len_wps):
            self.set_waypoint_velocity(lane.waypoints, idx, 0)

        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg): # Receive pose msg
        self.pose = msg

    def twist_cb(self, msg): ## Receive velocity msg
        self.twist = msg
        self.current_vel = self.twist.twist.linear.x

    def waypoints_cb(self, waypoints): #
        if not self.base_waypoints:
            self.base_waypoints = waypoints
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        if LOGWARN_MODE:
            rospy.logwarn("Cache base_waypoints once at first time")		

    def traffic_cb(self, msg): 
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_waypoint_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def throttle_cmd_cb(self, msg):
        self.throttle_cmd = msg.pedal_cmd
    
    def steering_cmd_cb(self, msg):
        self.steering_cmd = msg.steering_wheel_angle_cmd

    def brake_cmd_cb(self, msg):
        self.brake_cmd = msg.pedal_cmd

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
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
