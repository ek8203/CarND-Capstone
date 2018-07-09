
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, 
                vehicle_mass,
                fuel_capacity,
                brake_deadband,
                decel_limit,
                accel_limit,
                wheel_radius,
                wheel_base,
                steer_ratio,
                max_lat_accel,
                max_steer_angle):

        # TODO: Implement
        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

        # Steering control
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, 
                                            steer_ratio, 
                                            min_speed, 
                                            max_lat_accel, 
                                            max_steer_angle)

        # Speed control
        kp = 0.3
        ki = 0.1
        kd = 0.0
        throttle_min = 0.0
        throttle_max = 0.2
        self.throttle_controller = PID(kp, ki, kd, throttle_min, throttle_max)

        # Filter noisy velocity
        tau = 0.5   # cutoff freq = 1/(2*pi*tau)
        ts = 0.02   # sample time
        self.lpf = LowPassFilter(tau, ts)

        self.last_time = rospy.get_time()

    def control(self, current_velocity, linear_velocity, angular_velocity, dbw_enabled):
       # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.0
        brake = 0.0
        steer = 0.0
        
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.        
        
        # filter current velocity    
        current_velocity = self.lpf.filt(current_velocity)

        # get steering
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # velocity error
        error = linear_velocity - current_velocity
        self.last_velocity = current_velocity

        # sample time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # get throttle
        throttle = self.throttle_controller.step(error, sample_time)

        # set brake to 400 N*m to hold stopped vehicle
        brake = 0.
        if linear_velocity == 0. and current_velocity < 0.1:
            throttle = 0.
            brake = 400.
        # allpy brake to deccelerate    
        elif throttle < 0.1 and error < 0:  
            throttle = 0.
            decelerate = max(error, self.decel_limit)
            # torque in N*m
            brake = abs(decelerate) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steer
