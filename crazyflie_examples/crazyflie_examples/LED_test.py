# Transfer swarmalator phase[0,2*pi] to LED color
from crazyflie_py import Crazyswarm
import numpy as np

class LEDController:
    def __init__(self):
        # Initialize swarm
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.crazyflies = self.swarm.allcfs
        self.numBots = len(self.crazyflies.crazyflies)

        # Initialize phase
        self.phases = np.zeros(self.numBots)
        self.dPhase = np.zeros(self.numBots)

        # Initialize LED color
        self.LED_color = np.zeros((self.numBots, 3))

        # Initialize start time
        self.start_time = self.timeHelper.time()

    def run(self, duration=20):
        while self.timeHelper.time() - self.start_time < duration:
            self.update_phases()
            self.update_LED_color()
            self.timeHelper.sleep(0.1)

    def update_phases(self):
        self.dPhase = 0.1*np.ones(self.numBots)
        self.phases += self.dPhase
        # Apply modulo to keep phases in [0, 2π] range
        self.phases = self.phases % (2 * np.pi)
        
    def update_LED_color(self):
        for i in range(self.numBots):
            self.dPhase = 0.1*np.ones(self.numBots)
            self.phases += self.dPhase
            # Apply modulo to keep phases in [0, 2π] range
            self.phases = self.phases % (2 * np.pi)
            # Map phase (0 to 2π) to RGB color
            r = int(128 + 127 * np.cos(self.phases[i]))
            g = int(128 + 127 * np.cos(self.phases[i] + 2*np.pi/3))
            b = int(128 + 127 * np.cos(self.phases[i] + 4*np.pi/3))
            
            self.LED_color[i, 0] = r/255.0
            self.LED_color[i, 1] = g/255.0
            self.LED_color[i, 2] = b/255.0
            self.crazyflies.crazyflies[i].setLEDColor(
                self.LED_color[i, 0], 
                self.LED_color[i, 1], 
                self.LED_color[i, 2]
            )
            print(i)
        
def main():
    controller = LEDController()
    controller.run()

if __name__ == "__main__":
    main()
        
