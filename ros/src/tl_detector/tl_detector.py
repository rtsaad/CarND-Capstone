#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
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

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):

        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None

        

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        # List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        rospy.logwarn("Classifiers Loaded")

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0        
        self.lights = None        
        self.has_image = False
        self.over = True
        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

	self.num_image = 0

        #rospy.spin()
        self.loop()

    def loop(self):
        # Control the number of images checked per second
        rate = rospy.Rate(2) #2Hz
        
        
        while not rospy.is_shutdown():
            if not self.has_image:
                rate.sleep()
                continue
            
            # Check for new lights
            self.check_lights()

            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        # Save waypoints
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        self.list_lights = [[l.pose.pose.position.x, l.pose.pose.position.y] for l in self.lights]        

    def image_cb(self, msg):        
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        # Uncomment to use the state from emulator
	# Skip some of the input images to give more time to the classifier
	#if self.num_image < SKIP_IMAGES_NUMBER:
	#	self.num_image += 1
	#	return
	self.num_image = 0


        self.has_image = True
        self.camera_image = msg

    def check_lights(self):

        if not self.over:
            # ignore, not finished yet
            return
        self.over = False
        
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

        self.over = True


    def euclidean_distance(self, position1, position2):
        x = position1.x - position2.x
        y = position1.y - position2.y
        return math.sqrt((x*x) + (y*y))

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
           ## https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
           ## I believe this algorithm is not necessary here because it finds the closest pair of point that belongs to a set (both points are unknown). 
           ## For this case, we have to find the closest point from the set (waypoints) to the given pose, we should use a nearest neighbor algorithm or a spatial index.
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        if self.waypoints is None:
            # Waypoints not loaded yet, nothing to publish, ignore
            return 0
        
        i = 0
        index = -1
        index_min = 10000000
        a = (pose.position.x, pose.position.y)
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
        # O(n) execution
        for w in self.waypoints:
            b = (w.pose.pose.position.x, w.pose.pose.position.y)
            dist = dl(a,b)
            if dist < index_min:
                index = i
                index_min = dist
            i += 1
        return index

    def get_closest_point(self, list_points, point):
        """ The same as closet_waypoint but accepts list of points instead of waypoints msg type.       
        """
        
        if len(list_points) < 2:
            return 0
        
        i = 0
        index = -1
        index_min = 1000000
        a = (point[0], point[1])
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
        # O(n) execution
        for b in list_points:
            dist = dl(a,b)
            if dist < index_min:
                index = i
                index_min = dist
            i += 1
        return index
        

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

        # Reset guard variable
        self.has_image = False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
	image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        #Get classification
        return self.light_classifier.get_classification(image)

    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1
        light_wp_dist = -1
	
        if(self.pose and self.lights):
            # find the closest visible traffic light (if one exists)
            car_position = self.get_closest_waypoint(self.pose.pose)

	    # get closest light            
            light_position = self.get_closest_point(self.list_lights, [self.pose.pose.position.x,self.pose.pose.position.y])

            if light_position != -1:
                #check if it is in front
                 # Check if index point is in front of the car
                mapp = [self.lights[light_position].pose.pose.position.x, self.lights[light_position].pose.pose.position.y, self.lights[light_position].pose.pose.position.z]
                point = [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z]
                
                theta = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x,
                                                         self.pose.pose.orientation.y,
                                                         self.pose.pose.orientation.z,
                                                         self.pose.pose.orientation.w])[2]

                # Check orientation
                heading = math.atan2((mapp[1] - point[1]), (mapp[0] - point[0]))
                
                angle = abs(theta - heading)        
                
                if angle < math.pi/4:
                    # OK, in front                   
                    
                    # check distance from the closest traffic light
                    dist_l = self.euclidean_distance(self.pose.pose.position, self.lights[light_position].pose.pose.position)
                    
                    if dist_l < 300: #less 300 meters                    
                        # get closest stop position
                                
                        stop_position = self.get_closest_point(self.stop_line_positions, self.list_lights[light_position])
                        if stop_position != -1:                            
                            pose = Pose()
                            pose.position.x = self.stop_line_positions[stop_position][0]
                            pose.position.y = self.stop_line_positions[stop_position][1]
                            
                            light_wp =  self.get_closest_waypoint(pose)                            
                            light = light_wp
                            
                            state = self.get_light_state(light)
                            # Uncomment to use the state from emulator
                            #state = self.lights[light_position].state
                            return light_wp, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
