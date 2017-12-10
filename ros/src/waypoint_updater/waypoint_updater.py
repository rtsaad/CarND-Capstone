#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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

# Global Constants
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
KMPH_TO_MPS = 1000./(60.*60.)
MAX_BRAKE =  0.25
MAX_ACC   = 0.025

class WaypointUpdater(object):

    waypoints = None
    original_waypoints = None
    current_waypoint = None
    current_waypoint_index = None
    final_waypoints_pub = None
    max_speed = None
    
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        #Add a subscriber for /traffic_waypoint and /obstacle_waypoint below        
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: check if obstacle_waypoint is really necessary. Rostopic did not find this topic
        #rospy.Subscriber('/obstacle_waypoint', unknown message type, self.obstacle_cb)

        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.waypoints = None
        self.current_waypoint = None
        self.current_waypoint_index = None

        # Load global variables
        self.max_speed = rospy.get_param('/waypoint_loader/velocity') * KMPH_TO_MPS

        rospy.spin()


    ## Message Functions
    
    def pose_cb(self, msg):
        # Save current position
        self.current_waypoint = msg
        rospy.loginfo('New position received {}'.format(msg.header.seq))        
        # Publish waypoints ahead at every new pose
        self.waypoints_publish()        

    def waypoints_cb(self, msg):
        # Load waypoints
        self.waypoints = msg.waypoints
        self.original_waypoints = msg.waypoints
        rospy.loginfo('Waypoints loaded successfully ({})'.format(len(self.waypoints)))
        

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.        
        # Set speed for waypoints before red lights and stop points.
        # Recover waypoint stop position
        stop_waypoint_index = msg.data
        
        if stop_waypoint_index == -1:
            # No Red light ahead, recover original waypoitns
            self.waypoints = self.original_waypoints
            return
        
        # Compute distance to stop
        distance_full_stop = self.distance(self.waypoints, self.current_waypoint_index, stop_waypoint_index)
        # Compute linear slowdown
        current_speed = self.get_waypoint_velocity(self.waypoints[self.current_waypoint_index])
        previous_speed = current_speed
        linear_slowdown = current_speed/distance_full_stop
        for i in range(self.current_waypoint_index+1, stop_waypoint_index):
            short_distance = self.distance(self.waypoints, i, i+1)
            velocity = current_speed - (linear_slowdown*short_distance)
            if abs(previous_speed - velocity) > MAX_BRAKE:
                velocity = current_speed - MAX_BRAKE
            if velocity < 0:
                velocity = 0
            previous_speed = current_speed
            current_speed = velocity
            self.set_waypoint_velocity(self.waypoints, i, current_speed)            
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def waypoints_publish(self):
        if self.waypoints is None:
            # Waypoints not loaded yet, nothing to publish, ignore
            return

        # Get index for closest waypoint
        index = self.get_closest_waypoint(self.current_waypoint.pose.position.x,
                                          self.current_waypoint.pose.position.y,
                                          self.current_waypoint.pose.position.z)
        # Save current waypoint index
        self.current_waypoint_index = index
        # Publish N(LOOKAHEAD_WPS) waypoints ahead
        msg_pub = Lane()        
        msg_pub.header.stamp = rospy.Time(0)
        msg_pub.header.frame_id = self.current_waypoint.header.frame_id
        msg_pub.waypoints = self.waypoints[index:(index+LOOKAHEAD_WPS)]
        rospy.loginfo('Publish new waypoints sarting at index {}'.format(index))
        self.final_waypoints_pub.publish(msg_pub)


    ## Local Helpers
        
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
        index_min = 1000000
        a = (x,y,z)
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2  + (a[2]-b[2])**2)
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
