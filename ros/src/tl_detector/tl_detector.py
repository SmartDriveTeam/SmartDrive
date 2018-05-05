#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import PyKDL
from tf.transformations import euler_from_quaternion
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.light_classifier = None

        self.tlclasses_d = { TrafficLight.RED : "RED_LIGHT", TrafficLight.YELLOW:"YELLOW_LIGHT", TrafficLight.GREEN:"GREEN_LIGHT", TrafficLight.UNKNOWN:"UNKNOWN" }

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb) #Simulator has state of traffic light built in
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=32*10**6) #Camera data; set buffer to reduce lag
        sub7 = rospy.Subscriber('/image_raw', Image, self.raw_image_cb) #Image raw used for classifier for more data

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
	
	
        #setup stop line positions in TrafficLight-style object for use later on closestwaypoint
        self.stop_line_positions_poses = []
        for stop in self.config['stop_line_positions']:
            s = TrafficLight()
            s.pose.pose.position.x = stop[0]
            s.pose.pose.position.y = stop[1]
            s.pose.pose.position.z = 0
            self.stop_line_positions_poses.append(s)
	

        #Publishing
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.tl_detector_initialized_pub = rospy.Publisher('/tl_detector_initialized', Bool, queue_size=1)

        #Set up Classifier
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.tl_detector_initialized_pub.publish(Bool(True))
        rospy.loginfo('Traffic light detector initialized')
        rospy.spin()

    #Callbacks
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def raw_image_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        state = self.light_classifier.get_classification(cv_image)
        rospy.loginfo(self.tlclasses_d[ state ] )

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    '''def get_distance_between_poses( self, a, b ):
        return math.sqrt( (a.position.x - b.position.x)**2 + (a.position.y - b.position.y)**2 )
'''
    def get_closest_waypoint(self, pose, waypoints,  mode=None ):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypoints (list): the reference list of waypoints to search on
            mode: "nearest" -> returns nearest waypoint regardless of direction. 
                  "forward" -> returns nearest waypoint in forward direction

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        if waypoints==None or pose==None:
            #rospy.logerr("No waypoint list or pose specified in get_closest_waypoint")
            return -1

        #Find nearest
        max_range = 275
        min_dist = float("inf")
        min_idx = None

        for i, wp in enumerate(waypoints):
            dist = math.sqrt((wp.pose.pose.position.x - pose.position.x)**2 + (wp.pose.pose.position.y - pose.position.y)**2)

            if (dist < min_dist) and (dist < max_range):
                if (mode == None ):
                    min_dist = dist
                    min_idx = i
                elif( mode == "forward" ): 
                    po = pose.orientation         
                    wpo = wp.pose.pose.orientation  
                    wpp = wp.pose.pose.position
                    car_vector = PyKDL.Rotation.Quaternion(po.x,po.y,po.z,po.w) * PyKDL.Vector(1,0,0) 
                    wp_vector = PyKDL.Vector( wpp.x-pose.position.x, wpp.y-pose.position.y, 0 )

                    angle = np.arccos( PyKDL.dot( car_vector, wp_vector ) / car_vector.Norm() / wp_vector.Norm() )

                    if angle < np.pi/2:
                        min_dist = dist
                        min_idx = i
                        
        return min_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closer to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

    	# If any of these conditions are found, return none
        if self.pose is None:
            return -1, TrafficLight.UNKNOWN

        if self.waypoints is None:
            return -1, TrafficLight.UNKNOWN

        if self.light_classifier is None:
            return -1, TrafficLight.UNKNOWN

        #TODO find the closest visible traffic light (if one exists)
        light_idx = self.get_closest_waypoint( self.pose.pose, self.lights, "forward" )  # foward look

        if light_idx == None:
            return -1, TrafficLight.UNKNOWN

        # Find closest stop line waypoint index
        stop_line_idx    = self.get_closest_waypoint( self.lights[light_idx].pose.pose, self.stop_line_positions_poses ) # closest look
        # Check stop line in front of vehicle
        infront_idx = self.get_closest_waypoint( self.pose.pose, self.stop_line_positions_poses, "forward" )  # foward look
        # Make sure that they agree; if not return UNKNOWN
        if(stop_line_idx != infront_idx):
            return -1, TrafficLight.UNKNOWN

        # Get waypoint closest to light
        stop_waypoint_idx = self.get_closest_waypoint( 
                self.stop_line_positions_poses[stop_line_idx].pose.pose, 
                self.waypoints.waypoints )  

        if( stop_waypoint_idx == None ):
            return -1, TrafficLight.UNKNOWN

        use_detector_flag = True
   
        if use_detector_flag:
            state = self.get_light_state( self.lights[light_idx] )
        else:
            state = self.lights[light_idx].state  

        rospy.loginfo(self.tlclasses_d[ state ] )
        return stop_waypoint_idx, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
