import math
import numpy as np
import time
import threading
import sys
import select
from crazyflie_py import Crazyswarm

class Swarmalator:
    def __init__(self):
        # Initialization variables
        self.botRad = 0.15
        self.height = 1.0
        self.dt = 0.1
        self.A = 1
        self.B = 0.8
        self.J = 1
        self.K = 5

        # Parameters for the swarmalator
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.crazyflies = self.swarm.allcfs
        self.numBots = len(self.crazyflies.crazyflies)

        # Store the position and phase information
        self.positions = np.zeros((self.numBots, 3))
        self.phases = np.zeros(self.numBots)
        self.dPos = np.zeros((self.numBots, 3))
        self.dPhase = np.zeros(self.numBots)

        # Initialize the positions and phases
        self.initialize_positions()
        self.initialize_phases()
        
        # Flag to control execution
        self.running = False

    def initialize_positions(self):
        self.crazyflies.takeoff(targetHeight=self.height, duration=1.0+self.height)
        self.timeHelper.sleep(1.5+self.height)
        for i, cf in enumerate(self.crazyflies.crazyflies):
            pos = np.array(cf.initialPosition) + np.array([0, 0, self.height])
            cf.goTo(pos, 0, 1.0)
            self.positions[i] = pos

    def initialize_phases(self):
        self.phases = np.linspace(0, 2*np.pi, self.numBots)

    def monitor_for_keyboard_press(self):
        """Monitor for keyboard press and land when detected"""
        print("Press any key to stop and land...")
        while self.running:
            key = check_for_keypress()
            if key is not None:
                print(f"Key '{key}' pressed! Landing all Crazyflies...")
                self.stop_and_land()
                break
            time.sleep(0.1)

    def stop_and_land(self):
        """Stop execution and land all Crazyflies"""
        self.running = False
        # Land all crazyflies
        self.crazyflies.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(2.5)  # Wait for landing to complete
        print("All Crazyflies landed safely")

    def run(self, duration=60):
        """Run the swarmalator algorithm"""
        self.running = True
        
        # Start a thread to monitor for keyboard presses
        monitor_thread = threading.Thread(target=self.monitor_for_keyboard_press)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        # Your swarmalator algorithm logic here
        start_time = time.time()
        try:
            while self.running and (time.time() - start_time < duration):
                # Implement the swarmalator algorithm here
                for i in range(self.numBots):
                    # Initialize the derivatives
                    self.dPos[i] = np.zeros(3)
                    self.dPhase[i] = 0

                    # Compute the derivatives
                    for j in range(self.numBots):
                        if i != j:
                            # Compute the distance between the two bots (In 2D here)
                            dx = self.positions[j][0] - self.positions[i][0]
                            dy = self.positions[j][1] - self.positions[i][1]
                            dist = np.sqrt(dx**2 + dy**2)
                            # Calculate the absolute distance between the two bots
                            if dist < 2*self.botRad:
                                d = dist - 2*self.botRad
                            else:
                                d = dist*0.2    # This is to avoid the bots to collide
                            
                            # Compute the attractive force
                            F_attr = ( self.A + self.J*np.cos(self.phases[j] - self.phases[i]) )/dist
                            F_rep = self.B/(dist*d)
                            F_total = F_attr - F_rep

                            # Compute the velocity
                            self.dPos[i][0] += F_total * dx
                            self.dPos[i][1] += F_total * dy

                            # Compute the phase attraction
                            self.dPhase[i] += self.K * np.sin(self.phases[j] - self.phases[i])/dist

                # Update the positions and phases
                for i in range(self.numBots):
                    # Update the position
                    self.positions[i][0] += self.dPos[i][0] * self.dt / self.numBots
                    self.positions[i][1] += self.dPos[i][1] * self.dt / self.numBots
                    self.positions[i][2] = self.height

                    # Regularize and update the phase
                    self.phases[i] += self.dPhase[i] * self.dt / self.numBots
                    self.phases[i] = np.mod(self.phases[i], 2*np.pi)

                    # Update the Crazyflie
                    self.crazyflies.crazyflies[i].goTo(self.positions[i], 0, self.dt)

                # self.timeHelper.sleep(0.1)
            
            # If we exit normally (not by button press), land the Crazyflies
            if self.running:
                self.stop_and_land()
                
        except KeyboardInterrupt:
            # Also handle Ctrl+C
            print("Program interrupted! Emergency landing...")
            self.stop_and_land()
        
def check_for_keypress():
    """Non-blocking keyboard check"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None
        

def main():
    swarmalator = Swarmalator()
    swarmalator.run()
    swarmalator.stop_and_land()

if __name__ == "__main__":
    main()