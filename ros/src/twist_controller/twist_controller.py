from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, linear_velocity, angular_velocity, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        throttle = 1.0
	brake = 0.0
	steering = 0.0

	return throttle, brake, steering
