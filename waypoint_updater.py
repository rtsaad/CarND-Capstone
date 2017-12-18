#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):

    waypoints = None
    current_waypoint = None
    final_waypoints_pub = None
    
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        #Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # TODO: uncomment when traffic_light node is completed
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: check if obstacle_waypoint is really necessary. Rostopic did not find this topic
        #rospy.Subscriber('/obstacle_waypoint', unknown message type, self.obstacle_cb)

        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.waypoints = None
        self.current_waypoint = None

        rospy.spin()


    ## Message Functions
    
    def pose_cb(self, msg):
        # Save current position
        self.current_waypoint = msg
        rospy.loginfo('New position received {}'.format(msg.header.seq))        
        # Publish waypoints ahead at every new pose
        self.waypoints_publish()
        #pass

    def waypoints_cb(self, msg):
        # Load waypoints
        self.waypoints = msg.waypoints
        rospy.loginfo('Waypoints loaded successfully ({})'.format(len(self.waypoints)))        
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
        b = (0,0,0)
        index = 0
        index_min = 1000000
        a = (x,y,z)
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2  + (a[2]-b[2])**2)
        for w in self.waypoints:
            b = (w.pose.pose.position.x, w.pose.pose.position.y, w.pose.pose.position.z)
            dist = dl(a,b)
            if dist < index_min:
                index = i
                index_min = dist
            i += 1
        rospy.loginfo('Closest waypoint: %s',index)
        rospy.loginfo('DIST: %s', dist)
        rospy.loginfo('Currrent position x: %s',x)
        rospy.loginfo('Currrent position y: %s',y)
        rospy.loginfo('Currrent position z: %s',z)
        return index


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
