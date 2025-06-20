import math
import numpy as np
import time
import threading
import sys
import select
import csv
import os
import rclpy
from crazyflie_py import Crazyswarm
from crazyflie_examples.tracking import PositionTracker

class Swarmalator:
    def __init__(self):
        # Initialization variables
        self.botRad = 0.05
        self.height = 0.5
        self.minVel = -0.5
        self.maxVel = 0.5
        self.dt = 0.1
        self.A = 1
        self.B = 1
        self.J = 3
        self.K = 1

        # Parameters for the swarmalator
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.crazyflies = self.swarm.allcfs
        self.tracker = self.swarm.track
        self.numBots = len(self.crazyflies.crazyflies)
        self.frame_id = self.crazyflies.crazyfliesByName.keys()

        # Store the position and phase information
        self.status = np.full(self.numBots, False, dtype=bool)
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
        print("waiting for the tf positions to be published")
        
        time.sleep(1)

        self.update_positions()
        
        print("Starting to take off")
        for i, cf in enumerate(self.crazyflies.crazyflies):
            print("taking off")
            # cf.goTo(self.positions[i], 0, 1.0)
        # self.crazyflies.takeoff(targetHeight=self.height, duration=1.0+self.height)
        # self.timeHelper.sleep(1.5+self.height)

    def initialize_phases(self):
        # self.phases = np.linspace(0, 2*np.pi, self.numBots)
        self.phases = np.zeros(self.numBots)

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

    def update_positions(self):
        for i, frame in enumerate(self.frame_id):
            pos = self.tracker.get_position(to_frame=frame)
            if pos:
                x, y, z = pos
                self.status[i] = True
                self.positions[i] = [x, y, z]
                print("controlled", self.positions[i])
            else:
                self.status[i] = False
                print("lost_control")

    def update_LED_color(self):
        for i in range(self.numBots):
            r = int(128 + 127 * np.cos(self.phases[i])) / 255.0
            g = int(128 + 127 * np.cos(self.phases[i] + 2*np.pi/3)) / 255.0
            b = int(128 + 127 * np.cos(self.phases[i] + 4*np.pi/3)) / 255.0
            self.crazyflies.crazyflies[i].setLEDColor(r, g, b)


    def save_position(self):
        filename = "positions.csv"
        write_header = not os.path.exists(filename)

        with open(filename, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(["timestamp", "index", "x", "y", "z", "status"])
            for i in range(self.numBots):
                timestamp = time.time()  # current time in seconds since epoch
                x, y, z = self.positions[i]
                writer.writerow([timestamp, i, x, y, z, self.status[i]])

        # print("Saved to:", os.path.abspath(filename))

    def run(self, duration=30):
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
                # update positions:
                rclpy.spin_once(self.tracker, timeout_sec=0.01)
                self.update_positions()
                self.save_position()

                # Implement the swarmalator algorithm here
                for i in range(self.numBots):
                    if not self.status[i]:
                        # self.crazyflies.crazyflies[i].land(targetHeight=0.04, duration=1)
                        continue
                    # Initialize the derivatives
                    self.dPos[i] = np.zeros(3)
                    self.dPhase[i] = 0

                    # Compute the derivatives
                    for j in range(self.numBots):
                        if not self.status[i]:
                            continue
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
                    self.dPos[i][0] = max(self.minVel, min(self.dPos[i][0], self.maxVel))
                    self.dPos[i][1] = max(self.minVel, min(self.dPos[i][1], self.maxVel))
                    # print("dPos of Drone: ", i, " is: ", self.dPos[i])
                    self.positions[i][0] += self.dPos[i][0] * self.dt / self.numBots
                    self.positions[i][1] += self.dPos[i][1] * self.dt / self.numBots
                    self.positions[i][2] = self.height
                    # print("Position of Drone: ", i, " is: ", self.positions[i])

                    # Regularize and update the phase
                    self.phases[i] += self.dPhase[i] * self.dt / self.numBots
                    self.phases[i] = np.mod(self.phases[i], 2*np.pi)
                    # print("Phase of Drone: ", i, " is: ", self.phases[i])
                    # Update the Crazyflie
                    # self.crazyflies.crazyflies[i].cmdFullState(
                    #     self.positions[i],
                    #     self.dPos[i]/self.numBots,
                    #     np.array([0.0, 0.0, 0.0]),
                    #     0,
                    #     np.array([0.0, 0.0, 0.0])
                    # )
                    # self.crazyflies.crazyflies[i].goTo(self.positions[i], 0, 0.1)

                # self.update_LED_color()
                self.timeHelper.sleep(0.1)
            
            # If we exit normally (not by button press), land the Crazyflies
            if self.running:
                self.stop_and_land()
                
        except KeyboardInterrupt:
            # Also handle Ctrl+C
            print("Program interrupted! Emergency landing...")
            self.stop_and_land()
            self.tracker.shutdown()
        
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