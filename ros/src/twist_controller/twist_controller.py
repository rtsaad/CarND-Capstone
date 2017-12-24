import pid
import rospy
import lowpass
import yaw_controller

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


# steer pid parameter - Beautiful!
S_KP = 2.5
S_KI = 0.0006
S_KD = 3.5

# velocity pid parameters
V_KP = 30
V_KI = 3
V_KD = 0.3

ARBITRARY_LAG = 0.5

# Flags
use_steering_pid = False

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


        self.pid_velocity = pid.PID(kp = V_KP, ki = V_KI, kd = V_KD)



        self.yaw_controller = yaw_controller.YawController(
            self.wheel_base, self.steer_ratio, 0.0,
            self.max_lat_accel, self.max_steer_angle) # aLEX YAW CONTROLLER SET TO 0

         # max torque (1.0 throttle) and  max brake torque (deceleration lmt)
        self.max_acc_torque = self.vehicle_mass * self.max_acceleration * self.wheel_radius
        self.max_brake_torque = self.vehicle_mass * abs(self.decel_limit) * self.wheel_radius

        # 1st timestamp
        self.last_time = rospy.get_time()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        dbw_enabled = kwargs.get('dbw_enabled')
        v = kwargs.get('v')
        v_target = kwargs.get('v_target')
        w = kwargs.get('w')
        w_target = kwargs.get('w_target')

        if(not dbw_enabled):
            if use_steering_pid:
                self.pid_steer.reset()

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
            #corrective_steer = self.pid_steer.step(cte, ARBITRARY_LAG)
            #params, err = self.pid_steer.twiddle(cte)            
            corrective_steer = self.pid_steer.step(cte, dt)
        else:
            # will assume no lag
            corrective_steer = 0.0001


        # Speed Controller velocity error
        v_err = v_target - v
        T = self.pid_velocity.step(v_err,dt)
        # Adjust Throttle and Brake when speed is zero
        if v_target == 0 and v < 1:
            throttle = 0
            brake = 1
        else:
            # Compute Throttle and Brake
            throttle = T/self.max_acc_torque if T > 0. else 0.
            brake = abs(T) if T <= 0. else 0.

        # Yaw Controller
        yaw_steer = self.yaw_controller.get_steering( v_target, w_target, v)
        #steer = yaw_steer + corrective_steer
        steer = corrective_steer       

        # Return
        return throttle, brake, yaw_steer



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
        # Remember throttle comes in relative means
        T = self.vehicle_mass * a * self.wheel_radius

        return T
