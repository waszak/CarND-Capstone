from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, dbw_enabled, linear_velocity, angular_velocity, current_velocity):
        if not dbw_enabled:
            return 0, 0, 0
        return .5, 0., self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
