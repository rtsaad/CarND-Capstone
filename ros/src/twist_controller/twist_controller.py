import pid
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement


        # PID controllers

        max_abs_angle = kwargs.get('max_steer_angle')
        rospy.logwarn(max_abs_angle)

        self.pid_steer = pid.PID(kp=0.5, ki=0.025, kd=0.25,
                             mn = -max_abs_angle, mx = max_abs_angle)
        #self.pid_throttle
        #self.pid_brake
        fuel_capacity = kwargs.get('fuel_capacity')
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.vehicle_mass = self.vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = kwargs.get('wheel_radius')
        self.accel_limit = kwargs.get('accel_limit')
        self.decel_limit = kwargs.get('decel_limit')
        self.brake_deadband = kwargs.get('brake_deadband')
        self.max_acceleration = kwargs.get('max_acceleration')



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


        if(not dbw_enabled):
            self.pid_steer.reset()
            return 0.0

        # find the time duration and a new timestamp
        cur_time = rospy.get_time()
        dt = cur_time - self.last_time + 1e-6
        self.last_time = cur_time
        #cte = kwargs.get('cte')
        # use a pid controller to find the most suited steering angle
        #steer = self.pid_steer.step(cte,dt)


        # Speed Controller
        v_err = v_target - v

        # v = v0 + at
        # (v-v0) = a * t
        # a = d_v/dt
        a = v_err/dt
        #a = v_err/dt

        # acc limits to avoid jerk
        a =  min(self.accel_limit, a) if a >=0 else max(self.decel_limit,a)

        if abs(a)<self.brake_deadband:
            throttle, brake = 0. , 0.

        # Torque corresponds to throttle so...
        # lets find it
        # T = m . a . R
        # remember throttle comes in relative means
        T = self.vehicle_mass * a * self.wheel_radius
        throttle = T/self.max_acc_torque if T > 0 else 0
        brake = abs(T) if T <= 0 else 0


        #throttle = 1.
        #brake = 0.
        return throttle, brake, 0.
