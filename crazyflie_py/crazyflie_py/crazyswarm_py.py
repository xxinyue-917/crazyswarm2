import rclpy

from . import genericJoystick
from .crazyflie import CrazyflieServer, TimeHelper, PositionTracker


class Crazyswarm:

    def __init__(self):
        rclpy.init()

        self.allcfs = CrazyflieServer()
        self.tracker = PositionTracker('my_position_tracker')
        self.timeHelper = TimeHelper(self.allcfs)

        self.input = genericJoystick.Joystick(self.timeHelper)