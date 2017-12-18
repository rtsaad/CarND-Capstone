import pid
import rospy
import lowpass
import yaw_controller

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


# steer pid parameter
S_KP = 0.10
S_KI = 0.00
S_KD = 0.25

# velocity pid parameters
V_KP = 15.0
V_KI = 0.01
V_KD = 0.02


ARBITRARY_LAG = 0.5

# some flags
use_velocity_pid_only = False
use_velocity_corrective_pid = True
use_steering_pid = False
use_4times_brake = True


if use_velocity_corrective_pid:
    use_velocity_pid_only = False

if not use_velocity_pid_only and not use_velocity_corrective_pid:
    use_model_velocity_only = True
else:
    use_model_velocity_only = False




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
        if use_steering_pid:
            # create lowpass filters
            self.steer_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)

            self.pid_steer = pid.PID(kp= S_KP, ki= S_KI, kd= S_KD,
                                 mn = -max_abs_angle, mx = max_abs_angle)


        if not use_model_velocity_only:
            self.pid_velocity = pid.PID(kp = V_KP, ki = V_KI, kd = V_KD)



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
        w = kwargs.get('w')
        w_target = kwargs.get('w_target')

        if(not dbw_enabled):
            if use_steering_pid:
                self.pid_steer.reset()

            if not use_model_velocity_only:
                self.pid_throttle.reset()

            return 0.,0.,0.

        # find the time duration and a new timestamp
        cur_time = rospy.get_time()
        dt = cur_time - self.last_time  + 1e-6
        self.last_time = cur_time


        if use_steering_pid:
            cte = kwargs.get('cte')
            # this is a quite laggy environment so it's supposed to be corrected by another PID controller
            # as we have some model in yaw controller, we can just sum this corrective_steer after
            corrective_steer = self.pid_steer.step(cte, ARBITRARY_LAG)
            corrective_steer = self.steer_filter.filt(corrective_steer)
        else:
            # will assume no lag
            corrective_steer = 0.


        # Speed Controller velocity error
        v_err = v_target - v

        if use_model_velocity_only:
            T = self.velocity_model(v_err,dt)
        elif use_velocity_pid_only:
            T = self.pid_velocity.step(v_err,dt)
        elif use_velocity_corrective_pid:
            T = self.pid_velocity.step(v_err,ARBITRARY_LAG)
            T += self.velocity_model(v_err,dt)


        # Throttle and Brake
        throttle = T/self.max_acc_torque if T > 0. else 0.
        brake = abs(T) if T <= 0. else 0.


        yaw_steer = self.yaw_controller.get_steering( v_target, w_target, v)
        steer = yaw_steer + corrective_steer

        #rospy.logwarn("steer:")
        #rospy.logwarn(steer)
        #rospy.logwarn("T:")
        #rospy.logwarn(throttle)
        #rospy.logwarn("brake:")
        #rospy.logwarn(brake)

        if use_4times_brake:
            brake *=4


        return throttle, brake, steer



    def velocity_model(self, velocity_err, time):
        '''
        Calculates acceleration from physics formula and the the Torque
        :param velocity_err:
        :param time:
        :return: a Torque value (T)
        '''
        a = velocity_err/time
        # acc limits to avoid jerk
        a =  min(self.accel_limit, a) if a >=0. else max(self.decel_limit,a)

        if abs(a)<self.brake_deadband:
            return 0.0

        # Torque corresponds to throttle so...
        # lets find it
        # T = m . a . R
        # remember throttle comes in relative means
        T = self.vehicle_mass * a * self.wheel_radius

        return T
