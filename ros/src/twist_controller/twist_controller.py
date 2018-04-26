from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel,
                 max_steer_angle,
                 accel_limit,
                 decel_limit,
                 loop_frequency,
                 vehicle_mass,
                 wheel_radius):

        # TODO: Implement
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.max_steer_angle = max_steer_angle
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.steering_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(
            0.3, 0.1, 0.09, mn=decel_limit, mx=accel_limit)
        self.low_pass_filter = LowPassFilter(0.5, 0.02)
        self.last_timestamp = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset
            return 0., 0., 0.

        return 1., 0., 0.
########### Thought ##########
#       # To set up a lower limit
#        if math.fabs(linear_vel) < 0.1:
#            linear_vel.x = 0.
#        if math.fabs(angular_vel) < 0.001:
#            angular_vel.z = 0.
#        # To calculate the residual error for PID class
#        vel_err = (linear_vel - current_vel)/50.0

#        # Iteration for PID loop
#        if self.last_timestamp is not None:
#            # To get current time
#            time = rospy.get_time()      
#            # To compute time interval
#            dt = time - self.last_timestamp
#            self.last_timestamp = time
            
#            # PID class: it returns output for throttle & brake axes as a joint forward_backward axis
#            self.pid_class = PID(11.2, 0.05, 0.3, -accel_limit, accel_limit)    
#            forward_axis = self.pid_class.step(vel_err, dt)

#            # To obtain the steering value from YawController
#            steering = self.steering_controller.get_steering(linear_vel, angular_vel, current_vel)
#            # To update the steering by using steering pid loop  
            
#           return throttle, brake, steer
#        else:
#            # To update the last time stamp and return zeroes tuple
#            self.last_timestamp = rospy.get_time()
#            return 0., 0., 0.    
