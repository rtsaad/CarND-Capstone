import pid
import rospy
import lowpass
import yaw_controller

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        max_abs_angle = kwargs.get('max_steer_angle')
        #rospy.logwarn(max_abs_angle)

        fuel_capacity = kwargs.get('fuel_capacity')
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.vehicle_mass = self.vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = kwargs.get('wheel_radius')
        self.accel_limit = kwargs.get('accel_limit')
        self.decel_limit = kwargs.get('decel_limit')
        self.brake_deadband = kwargs.get('brake_deadband')
        self.max_acceleration = kwargs.get('max_acceleration')
        self.wheel_base = kwargs.get('wheel_base')
        self.steer_ratio = kwargs.get('steer_ratio')
        self.max_steer_angle = kwargs.get('max_steer_angle')
        self.max_lat_accel = kwargs.get('max_lat_accel')

        # PID controllers
        # create lowpass filters
        self.steer_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)

        self.pid_steer = pid.PID(kp=1., ki=0.025, kd=0.25,
                             mn = -max_abs_angle, mx = max_abs_angle)



        self.yaw_controller = yaw_controller.YawController(
            self.wheel_base, self.steer_ratio, 1.0,
            self.max_lat_accel, self.max_steer_angle)

         # max torque (1.0 throttle) and  max brake torque (deceleration lmt)
        self.max_acc_torque = self.vehicle_mass * self.max_acceleration * self.wheel_radius
        self.max_brake_torque = self.vehicle_mass * abs(self.decel_limit) * self.wheel_radius

        # 1st timestamp
        self.last_time = rospy.get_time()



    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #return 1.,0.0,0.0
        dbw_enabled = kwargs.get('dbw_enabled')
        v = kwargs.get('v')
        v_target = kwargs.get('v_target')
        w_target = kwargs.get('w_target')

        if(not dbw_enabled):
            #self.pid_steer.reset()
            #self.pid_throttle.reset()
            return 0.,0.,0.

        # find the time duration and a new timestamp
        cur_time = rospy.get_time()
        dt = cur_time - self.last_time  + 1e-6
        self.last_time = cur_time
        cte = kwargs.get('cte')
        # use a pid controller to find the most suited steering angle
        steer = self.pid_steer.step(cte,dt)

        # Speed Controller
        v_err = v_target - v

        # v = v0 + at
        # (v-v0) = a * t
        # a = d_v/dt
        a = v_err/dt
        #a = v_err/dt

        # acc limits to avoid jerk
        a =  min(self.accel_limit, a) if a >=0. else max(self.decel_limit,a)

        if abs(a)<self.brake_deadband:
            throttle, brake = 0. , 0.

        # Torque corresponds to throttle so...
        # lets find it
        # T = m . a . R
        # remember throttle comes in relative means
        T = self.vehicle_mass * a * self.wheel_radius
        throttle = T/self.max_acc_torque if T > 0. else 0.
        brake = abs(T) if T <= 0. else 0.


        #v1 = v+ a* dt
        v1 = v_target
        if v1< 0.0:
            v1 = 0.0
        yaw_steer = self.yaw_controller.get_steering(
           v1, w_target, v)

        steer = 0. #just to see yaw steer effects
        steer += yaw_steer

        rospy.logwarn("steer:")
        rospy.logwarn(steer)
        #rospy.logwarn("T:")
        #rospy.logwarn(throttle)
        #rospy.logwarn("brake:")
        #rospy.logwarn(brake)


        return throttle, brake, steer
