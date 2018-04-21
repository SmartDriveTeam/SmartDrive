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
