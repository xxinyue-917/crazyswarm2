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
        self.J = 1
        self.K = 0

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
        self.target_positions = np.zeros((self.numBots, 3))
        self.phases = np.zeros(self.numBots)
        self.dPos = np.zeros((self.numBots, 3))
        self.dPhase = np.zeros(self.numBots)


        # Initialize the positions and phases
        self.initialize_positions()
        self.initialize_phases()

        self.running = False
        
        # Flag to control execution

    def initialize_positions(self):
        print("waiting for the tf positions to be published")
        
        time.sleep(1)
        while not all(self.status):
            self.update_positions()
            self.timeHelper.sleep(0.1)
        
        print("Starting to take off")
        self.crazyflies.takeoff(targetHeight=self.height, duration=1.0+self.height)
        self.timeHelper.sleep(1.5+self.height)
        #update target_positions
        self.target_positions = [pos.copy() for pos in self.positions]
        for pos in self.target_positions:
            pos[2] = self.height
        
        for i, cf in enumerate(self.crazyflies.crazyflies):
            print("taking off")
            cf.goTo(self.target_positions[i], 0, 1.0)
        
        self.wait_until_reached()
        self.timeHelper.sleep(0.1)

    def initialize_phases(self):
        self.phases = np.linspace(0, 2*np.pi, self.numBots)
        # self.phases = np.zeros(self.numBots)

    def wait_until_reached(self, tolerance=0.15, timeout=10.0):
        """
        Waits until all drones are within a specified tolerance of their target positions.

        Args:
            tolerance: Distance threshold to consider the drone has reached the target.
            timeout: Maximum time to wait in seconds.

        Returns:
            True if all drones reached their targets within timeout, False otherwise.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            all_within = True
            self.update_positions()
            for i in range(len(self.positions)):
                if not self.status[i]:
                    continue
                current_pos = np.array(self.positions[i])
                target_pos = np.array(self.target_positions[i])
                distance = np.linalg.norm(current_pos - target_pos)
                if distance > tolerance:
                    all_within = False
                    break
            if all_within:
                break
            self.timeHelper.sleep(0.1)
        print("reached")


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
        rclpy.spin_once(self.tracker, timeout_sec=0.01)
        for i, frame in enumerate(self.frame_id):
            pos = self.tracker.get_position(to_frame=frame)
            if pos:
                x, y, z = pos
                self.status[i] = True
                self.positions[i] = [x, y, z]
                # print("controlled", self.positions[i])
            else:
                self.status[i] = False
                # print("lost_control")u
    # def update_positions(self):
    #     start_time = time.time()
    #     collected_positions = [[] for _ in self.frame_id]

    #     # Collect all TFs for 0.1 seconds
    #     while time.time() - start_time < 0.5:
    #         rclpy.spin_once(self.tracker, timeout_sec=0.01)
    #         for i, frame in enumerate(self.frame_id):
    #             pos = self.tracker.get_position(to_frame=frame)
    #             if pos:
    #                 collected_positions[i].append(pos)

    #     # Process collected positions
    #     for i, pos_list in enumerate(collected_positions):
    #         if len(pos_list) == 0:
    #             self.status[i] = False
    #             continue

    #         # Convert to numpy for processing
    #         pos_array = np.array(pos_list)

    #         # Compute mean and standard deviation
    #         mean = np.mean(pos_array, axis=0)
    #         std = np.std(pos_array, axis=0)

    #         # Filter out outliers (z-score > 2)
    #         z_scores = np.abs((pos_array - mean) / (std + 1e-6))  # add epsilon to prevent division by 0
    #         non_outliers = np.all(z_scores < 2.0, axis=1)
    #         filtered_positions = pos_array[non_outliers]

    #         if len(filtered_positions) == 0:
    #             self.status[i] = False
    #         else:
    #             avg_pos = np.mean(filtered_positions, axis=0)
    #             self.positions[i] = avg_pos.tolist()
    #             self.status[i] = True

    def update_LED_color(self):
        for i in range(self.numBots):
            r = int(128 + 127 * np.cos(self.phases[i])) / 255.0
            g = int(128 + 127 * np.cos(self.phases[i] + 2*np.pi/3)) / 255.0
            b = int(128 + 127 * np.cos(self.phases[i] + 4*np.pi/3)) / 255.0
            self.crazyflies.crazyflies[i].setLEDColor(r, g, b)


    def save_position(self):
        filename = f"A_{self.A}_B_{self.B}_K{self.K}_J{self.J}_numBots{self.numBots}.csv"
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
        index = 0
        try:
            while self.running and (time.time() - start_time < duration):
                # update positions:
                self.update_positions()
                self.save_position()
                self.target_positions = [pos.copy() for pos in self.positions]

                # Implement the swarmalator algorithm here
                for i in range(self.numBots):
                    # Initialize the derivatives
                    self.dPos[i] = np.zeros(3)
                    self.dPhase[i] = 0

                    # Compute the derivatives
                    for j in range(self.numBots):
                        if not self.status[i]:
                            continue
                        if i != j:
                            # Compute the distance between the two bots (In 2D here)
                            dx = self.target_positions[j][0] - self.target_positions[i][0]
                            dy = self.target_positions[j][1] - self.target_positions[i][1]
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
                    self.target_positions[i][0] += self.dPos[i][0] * self.dt / self.numBots
                    self.target_positions[i][1] += self.dPos[i][1] * self.dt / self.numBots
                    self.target_positions[i][2] = self.height
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
                    self.crazyflies.crazyflies[i].goTo(self.target_positions[i], 0, 0.1)

                self.update_LED_color()
                # self.wait_until_reached()
                self.timeHelper.sleep(0.1)
                print(index, "cycle complete")
                index += 1
            
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