#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import time
import math
import tf
import numpy as np
import copy

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

# Global Constants
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
KMPH_TO_MPS = 1000./(60.*60.)

class WaypointUpdater(object):

    waypoints = None
    waypoints_changed = True
    original_waypoints = None
    current_waypoint = None
    current_waypoint_index = None
    previous_waypoint_index = None
    final_waypoints_pub = None
    max_speed = None
    
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribe to topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)   
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)        
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)  
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.waypoints = None
        self.current_waypoint = None
        self.current_waypoint_index = None
        self.previous_waypoint_index = None
        self.current_waypoint_index_adjusted = None
        self.last_stop_index = -1
        self.velocity = None        
        self.over = True
        
        # Load global variables
        self.max_speed = rospy.get_param('/waypoint_loader/velocity') * KMPH_TO_MPS

        #rospy.spin()
        self.loop()


    def loop(self):
        rate = rospy.Rate(25) # 50Hz
        
        
        while not rospy.is_shutdown():
            if self.waypoints is None or self.current_waypoint is None:
                #rate.sleep()
                continue
            # Publish waypoints ahead at every new pose
            self.waypoints_update()
            self.waypoints_publish()

            rate.sleep()


    ## Message Functions

    def velocity_cb(self,msg):        
        self.velocity = msg.twist.linear.x
        if self.velocity < 0.1:
            self.velocity = 0
    
    def pose_cb(self, msg):
        # Save current position
        self.current_waypoint = msg        
        

    def waypoints_cb(self, msg):
        # Load waypoints        
        waypoints = msg.waypoints
        # adjust max speed
        for i in range(len(waypoints)):
            if self.get_waypoint_velocity(waypoints[i]) > self.max_speed:
                self.set_waypoint_velocity(waypoints, i, self.max_speed)
        self.waypoints = waypoints
        self.original_waypoints = copy.deepcopy(waypoints)
        rospy.logwarn('Waypoints loaded successfully ({})'.format(len(self.waypoints)))
        
    
    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.        
        # Set speed for waypoints before red lights and stop points.
        # Recover waypoint stop position       

        stop_waypoint_index = msg.data        

        # Check if its a new index point        
        if self.last_stop_index ==  stop_waypoint_index:            
            return
        
        should_brake = True

        if self.current_waypoint_index is None or self.waypoints is None or self.original_waypoints is None:
            # Ignore
            return

        # Get current Speed
        current_speed = self.velocity
        if current_speed <= 0:
            # Car must be parked
            # Small speed to reposition car
            current_speed = 2.8 #self.get_waypoint_velocity(self.waypoints[self.current_waypoint_index])/3

        distance_full = 101
        if stop_waypoint_index == -1:
            # No Red light ahead, recover original waypoitns
            # Accelerate
            lookahead = LOOKAHEAD_WPS/3
            stop_waypoint_index =  self.current_waypoint_index + lookahead
            if stop_waypoint_index > len(self.waypoints):
                stop_waypoint_index = len(self.waypoints)
            should_brake = False
            distance_full = self.distance(self.waypoints, self.current_waypoint_index, stop_waypoint_index-1)
            # Accelerates in two steps
            steps = 4
            
        else:
            # Compute distance to stop
            distance_full = self.distance(self.waypoints, self.current_waypoint_index, stop_waypoint_index-1)
            # Check time to brake with decelerating 0.9 m/s2 
            if  distance_full/current_speed > current_speed/0.8 and current_speed > 2.8:
                #rospy.logwarn(" Ignore {} {}".format(current_speed/0.9, distance_full/current_speed))
                #ignore
                return

            # Set the number of slow down steps
            steps = float(int(distance_full/10))
            if steps == 0:
                steps = 1

        # Update last index to avoid multiple calls
        if not should_brake:
            self.last_stop_index =  -1
        else:
            self.last_stop_index =  stop_waypoint_index
        
                
        # Compute linear acceleration/slowdown  
        # Recover original waypoint speed
        local_max_speed = self.get_waypoint_velocity(self.original_waypoints[stop_waypoint_index])

        if not should_brake and current_speed >= self.max_speed:            
            # Ignore, no red lights and speed is already high
            return
        
        # Step slow down or acceleration 
        linear_acceleration = current_speed/steps
        if not should_brake:
            linear_acceleration = (local_max_speed)/steps
        distance_divider = (distance_full/steps)

        if distance_divider < 1:
            distance_divider = 1
        
        # Update waypoints
        for i in range(self.current_waypoint_index, stop_waypoint_index+1):
            # Compute partial speed until stop
            velocity = 0
            short_distance = self.distance(self.waypoints, self.current_waypoint_index, i)
            multi = int(short_distance/distance_divider)
            diff_velocity = float(multi + 1) * linear_acceleration             
            if should_brake:
                # Braking
                velocity = current_speed - diff_velocity
                if velocity < 0:
                    velocity = 0
            else:
                # Accelerating
                velocity = current_speed + diff_velocity
                if velocity > self.max_speed:
                    velocity = self.max_speed
                elif velocity < 1.5:
                    velocity = 1.5                
            #rospy.logwarn("Set speed {} {} {} {} {}".format(i, velocity, multi, linear_acceleration, short_distance))
            self.set_waypoint_velocity(self.waypoints, i, velocity)

        # Check for stops positons of previous red lights
        if not should_brake:
            over = False
            index = stop_waypoint_index+1
            while not over:
                wp_speed = self.get_waypoint_velocity(self.original_waypoints[index])
                wp_current_speed = self.get_waypoint_velocity(self.waypoints[index])
                if wp_current_speed < wp_speed:
                    self.set_waypoint_velocity(self.waypoints, index, wp_speed)
                    #rospy.logwarn("Set speed {} {}".format(index, wp_speed))
                    index +=1                    
                else:
                    #rospy.logwarn("Index not speed {} {} {}".format(index, wp_current_speed, wp_speed))
                    over = True
        

        # Force to Publish new waypoints
        self.waypoints_changed = True        

        if should_brake:
            rospy.logwarn("Red Light position detected")
        else:
            rospy.logwarn("Green Light position detected")

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def waypoints_update(self):
        if self.waypoints is None:
            # Waypoints not loaded yet, nothing to publish, ignore
            return

        # Get index for closest waypoint
        index = self.get_closest_waypoint(self.current_waypoint.pose.position.x,
                                          self.current_waypoint.pose.position.y,
                                          self.current_waypoint.pose.position.z)
        
        # Save current waypoint index       
        self.previous_waypoint_index = self.current_waypoint_index 
        self.current_waypoint_index = index
                
        # Check if index point is in front of the car
        mapp = [self.waypoints[index].pose.pose.position.x, self.waypoints[index].pose.pose.position.y, self.waypoints[index].pose.pose.position.z]
        point = [self.current_waypoint.pose.position.x, self.current_waypoint.pose.position.y, self.current_waypoint.pose.position.z]
        
        theta = tf.transformations.euler_from_quaternion([self.current_waypoint.pose.orientation.x,
                                                         self.current_waypoint.pose.orientation.y,
                                                         self.current_waypoint.pose.orientation.z,
                                                         self.current_waypoint.pose.orientation.w])[2]
    
        

        X, Y = None,None
        w = self.waypoints[index]
        s_x = w.pose.pose.position.x - point[0]
        s_y = w.pose.pose.position.y - point[1]
        X = s_x* math.cos(-theta) - s_y*math.sin(-theta)
        Y = s_x* math.sin(-theta) + s_y*math.cos(-theta)
            
        if X < 0 or Y < 0:
            index += 1

        # Check orientation
        heading = math.atan2((mapp[1] - point[1]), (mapp[0] - point[0]))

        angle = abs(theta - heading)        

        if angle > math.pi/4:
            index += 1

        self.current_waypoint_index_adjusted = index
            

    def waypoints_publish(self):        
        
        if self.current_waypoint_index_adjusted is None:
            return

        index = self.current_waypoint_index_adjusted
        
        if index > len(self.waypoints):
            index = 0

        index_end = index+LOOKAHEAD_WPS
        if index_end > len(self.waypoints):
            index_end = len(self.waypoints)

        # Publish N(LOOKAHEAD_WPS) waypoints ahead
        msg_pub = Lane()
        msg_pub.header.frame_id = 'waypoints_ahead'
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.waypoints = self.waypoints[index:index_end]
        
        self.final_waypoints_pub.publish(msg_pub)
        self.waypoints_changed = False       

    ## Local Helpers
        
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def set_waypoint_yaw(self, waypoints, waypoint, yaw):
        waypoints[waypoint].twist.twist.angular.z = yaw

    def distance(self, waypoints, wp1, wp2):
        # Get distance between two waypoints
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def get_closest_waypoint(self, x, y, z):
        # Find the index of the closest waypoint       
        #TODO: use an RTREE(Python) to improve performance, not sure how to handle dependencies with ROS
        i = 0
        index = 0
        index_min = 100000000.
        a = (x,y,z)        
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)#  + (a[2]-b[2])**2)
        # O(n) execution
        for w in self.waypoints:
            b = (w.pose.pose.position.x, w.pose.pose.position.y, w.pose.pose.position.z)
            dist = dl(a,b)
            if dist < index_min:
                index = i
                index_min = dist
            i += 1
        return index

    
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
