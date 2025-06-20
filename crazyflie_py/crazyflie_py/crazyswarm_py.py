import rclpy

from . import genericJoystick
import threading
from .crazyflie import CrazyflieServer, TimeHelper, TFPositionNode


class Crazyswarm:

    def __init__(self):
        rclpy.init()
        print("started")
        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)
        self.track = TFPositionNode()
        self.input = genericJoystick.Joystick(self.timeHelper)