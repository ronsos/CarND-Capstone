from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        vehicle_mass   = kwargs['vehicle_mass']
	fuel_capacity  = kwargs['fuel_capacity']
	brake_deadband = kwargs['brake_deadband']
	decel_limit    = kwargs['decel_limit']
	accel_limit    = kwargs['accel_limit']
	wheel_radius   = kwargs['wheel_radius']
	wheel_base     = kwargs['wheel_base']
	steer_ratio    = kwargs['steer_ratio']
	max_lat_accel  = kwargs['max_lat_accel']
	max_steer_angle = kwargs['max_steer_angle']	

	pass

    def control(self, linear_velocity, angular_velocity, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        throttle = 1.0
	brake = 0.0
	steering = 0.0

	return throttle, brake, steering
