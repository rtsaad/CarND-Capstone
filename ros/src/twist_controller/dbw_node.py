#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from math import sin, cos
from tf import transformations
import numpy as np

from twist_controller import Controller


# euler
def get_euler(pose):
    """Returns the roll, pitch yaw angles from a Quaternion """
    return transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])


# Calculates the cross track error of the car
def calculate_cte(**kwargs):
    #TODO
    X, Y = [], []
    pose = kwargs.get('curpose')
    wp = kwargs.get('waypoints')
    maxpoints = kwargs.get('maxpoints')
    yaw = get_euler(pose)[2]
    x0 = pose.position.x
    y0 = pose.position.y

    # Shift and rotate waypoints
    for i in range(maxpoints):
        s_x = wp[i].pose.pose.position.x - x0
        s_y = wp[i].pose.pose.position.y - y0
        X.append(s_x* cos(-yaw) - s_y*sin(-yaw))
        Y.append(s_x* sin(-yaw) + s_y*cos(-yaw))

    #rospy.logwarn(yaw)

    A = np.polyfit(X,Y,3)
    return np.polyval(A,0.5)



'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        max_acceleration = rospy.get_param('~max_acceleration', 1.5)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', 
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', 
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
              

        # Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        kwargs = {
            "vehicle_mass": vehicle_mass,
            "fuel_capacity": fuel_capacity,
            "brake_deadband": brake_deadband,
            "decel_limit": decel_limit,
            "accel_limit": accel_limit,
            "wheel_radius": wheel_radius,
            "wheel_base": wheel_base,
            "steer_ratio": steer_ratio,
            "max_lat_accel": max_lat_accel,
            "max_steer_angle": max_steer_angle,
            "max_acceleration": max_acceleration
        }

        #rospy.logwarn(kwargs)
        self.controller = Controller(**kwargs)


        # Subscribe to all the topics you need to
        self.dbw_enabled = True
        self.waypoints = None
        self.twist = None
        self.velocity = None
        self.w = None
        self.current_pose = None
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(25) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)

            #rospy.logwarn(self.dbw_enabled)

            # If node isnt able to receive waypoints
            if self.waypoints is None:
                #rate.sleep()
                continue

            # If node isnt able to receive twist
            if self.twist is None:
                #rate.sleep()
                continue
            if self.current_pose is None:
                continue
            if self.velocity is None:
                continue

            #num_of_waypoints = len(self.waypoints)
            #rospy.logwarn(self.current_pose)


            # Calculate cte (to find steer)
            cte_args = {"waypoints": self.waypoints, "curpose": self.current_pose, "maxpoints": 10}
            #cte = calculate_cte(**cte_args)
            cte = self.twist.angular.z

            #rospy.logwarn(cte)
            # Find: velocity, target velocity, ...
            v_target = self.twist.linear.x
            v = self.velocity
            w_target = self.twist.angular.z
            w = self.w

            # Control

            controller_args = {"cte": cte,
                               "dbw_enabled": self.dbw_enabled,
                               "v": v,
                               "v_target": v_target,
                               "w_target": w_target,
                               "w": w}
            throttle, brake, steer = self.controller.control(**controller_args)

            # Publish control data to ROS topics
            # Is Manual Driving


            #rospy.logwarn(v)
            #rospy.logwarn(v_target)
            #rospy.logwarn(throttle)
            #rospy.logwarn(brake)


            if self.dbw_enabled:
                #rospy.logwarn("OK")
                self.publish(throttle,brake,steer)
            #else:
                #rospy.logwarn("Manual")


            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def waypoints_cb(self, msg):
        # TODO: Change it later, Simple implementation only to test the partial waypoint node.
        #first_w = msg.waypoints[0]
        #rospy.loginfo('Received waypoints of size {}'.format(len(msg.waypoints)))
        #self.publish(1, 0, 0)
        self.waypoints = msg.waypoints

    def dbw_enabled_cb(self,msg):
        self.dbw_enabled = bool(msg.data)

    def twist_cb(self,msg):
        self.twist = msg.twist

    def velocity_cb(self,msg):
        self.velocity = msg.twist.linear.x
        self.w = msg.twist.angular.z

    def current_pose_cb(self,msg):
        self.current_pose = msg.pose


if __name__ == '__main__':
    DBWNode()
