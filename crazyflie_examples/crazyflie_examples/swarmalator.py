#!/usr/bin/env python3

"""
Swarmalator implementation for Crazyswarm2.
This script implements the static sync collective behavior of the Swarmalator model
on multiple Crazyflie drones.

Reference:
O'Keeffe, K. P., Hong, H., & Strogatz, S. H. (2017).
Oscillators that sync and swarm.
Nature Communications, 8(1), 1504.
"""

import math
import numpy as np
import time
import threading
from crazyflie_py import Crazyswarm
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class SwarmalatorController:
    def __init__(self):
        """Initialize the Swarmalator controller."""
        # Swarmalator model parameters
        self.J = 1.0     # Spatial attraction strength (positive for attraction)
        self.K = 0     # Phase coupling strength (positive for synchronization)
        
        # Flight parameters
        self.height = 1.0  # Flight height in meters
        self.radius = 1.0  # Maximum radius of the swarm in meters
        self.max_vel = 0.15  # Max velocity (m/s)
        self.dt = 0.02     # Control loop time step (increased to 50Hz from 10Hz)
        
        # Position update frequency can be different from control loop frequency
        self.position_update_ratio = 1  # Update position every N control cycles
        self.position_update_counter = 0
        
        # Control flags
        self.is_running = False
        self.continuous_position_update = True  # Enable continuous position updates
        self.continuous_update_rate = 30.0  # Hz - can be different from main control rate
        
        # Initialize Crazyswarm
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.crazyflies = self.swarm.allcfs.crazyflies
        
        # Check if we're in simulation mode
        self.is_simulation = True
        try:
            # Try to access a node property that would be available in real mode
            if hasattr(self.swarm.allcfs, '_node'):
                node = self.swarm.allcfs._node
                self.is_simulation = 'use_sim_time' in node.get_parameter_names()
                self.node = node  # Store node reference for creating timers
        except:
            # If there's any error, assume we're in simulation
            self.is_simulation = True
        
        print(f"Running in {'simulation' if self.is_simulation else 'hardware'} mode")
        
        # Store positions, phases, and LEDs for each Crazyflie
        self.num_cfs = len(self.crazyflies)
        self.positions = np.zeros((self.num_cfs, 3))
        self.velocities = np.zeros((self.num_cfs, 3))
        self.phases = np.zeros(self.num_cfs)
        self.phase_velocities = np.zeros(self.num_cfs)
        self.target_positions = np.zeros((self.num_cfs, 3))  # Store target positions
        
        # Position lock for thread safety
        self.position_lock = threading.Lock()
        # ROS spin lock to prevent multiple threads from calling spin_once simultaneously
        self.ros_spin_lock = threading.Lock()
        
        # Initially distribute phases uniformly instead of randomly
        # This creates evenly spaced phases across the circle (0 to 2π)
        self.phases = np.linspace(0, 2 * np.pi, self.num_cfs, endpoint=False)
        
        # Position update thread
        self.position_thread = None
        
        # Natural repulsion parameters for collision avoidance
        self.repulsion_strength = 2  # Strength of natural repulsion
        self.repulsion_scale = 0.3     # Distance scale for repulsion effect

    def initialize_positions(self):
        """Initialize the positions of the Crazyflies in a circle."""
        # Take off to the specified height
        self.swarm.allcfs.takeoff(targetHeight=self.height, duration=2.0)
        self.timeHelper.sleep(2.0)
        
        # # Distribute drones in a circle
        # for i, cf in enumerate(self.crazyflies):
        #     angle = 2 * np.pi * i / self.num_cfs
        #     radius = self.radius * 0.7  # Start at 70% of max radius
            
        #     # Calculate position (x, y maintain the same z height)
        #     x = radius * np.cos(angle)
        #     y = radius * np.sin(angle)
        #     z = self.height
            
        #     # Store initial position
        #     initial_pos = np.array(cf.initialPosition)
        #     target_pos = initial_pos + np.array([x, y, z - initial_pos[2]])
            
        #     # Send drone to initial position
        #     cf.goTo(target_pos, 0, 1)
        
        # Wait for all drones to reach their positions
        # self.timeHelper.sleep(0.1)
        
        # Record current positions (in simulation these are command positions)
        # for i, cf in enumerate(self.crazyflies):
        #     # In a real implementation, you would get actual positions from motion capture
        #     # Here we just use the commanded positions
        #     self.positions[i] = np.array(cf.initialPosition) + np.array([
        #         self.radius * 0.7 * np.cos(2 * np.pi * i / self.num_cfs),
        #         self.radius * 0.7 * np.sin(2 * np.pi * i / self.num_cfs),
        #         self.height - cf.initialPosition[2]
        #     ])

    def update_leds(self):
        """Update LED colors based on the phase of each Crazyflie."""
        # In simulation mode, don't try to set LEDs to avoid warnings
        if self.is_simulation:
            # Just log once for information
            if not hasattr(self, '_led_warning_shown'):
                print("Note: LED control disabled in simulation mode")
                self._led_warning_shown = True
            return
            
        # Only attempt to control LEDs in hardware mode
        for i, cf in enumerate(self.crazyflies):
            # Map phase (0 to 2π) to RGB color
            r = int(128 + 127 * np.cos(self.phases[i]))
            g = int(128 + 127 * np.cos(self.phases[i] + 2*np.pi/3))
            b = int(128 + 127 * np.cos(self.phases[i] + 4*np.pi/3))
            
            try:
                # Set LED color (if LED deck is installed)
                # Only use the hardware parameter format
                cf.setParam("ring", "effect", 7)  # Solid color
                cf.setParam("ring", "solidRed", r)
                cf.setParam("ring", "solidGreen", g)
                cf.setParam("ring", "solidBlue", b)
            except Exception as e:
                # Only show LED error once per drone to avoid flooding console
                if not hasattr(cf, '_led_error_shown'):
                    print(f"Warning: Could not set LED color for drone {i}: {e}")
                    print("LED control will be disabled for this drone")
                    cf._led_error_shown = True

    def compute_swarmalator_dynamics(self):
        """Compute the Swarmalator dynamics for position and phase."""
        position_derivatives = np.zeros((self.num_cfs, 3))
        phase_derivatives = np.zeros(self.num_cfs)
        
        # For each Crazyflie
        for i in range(self.num_cfs):
            # Compute interactions with all other Crazyflies
            for j in range(self.num_cfs):
                if i != j:
                    # Calculate vector and distance between drones
                    r_ij = self.positions[j] - self.positions[i]
                    r_ij[2] = 0  # Only consider x-y plane for distance
                    distance = np.linalg.norm(r_ij)
                    
                    # Avoid division by zero
                    if distance < 0.001:
                        distance = 0.001
                    
                    # Calculate spatial interaction term
                    # 1. Attraction based on phase similarity (core Swarmalator behavior)
                    attraction = (1 + self.J * np.cos(self.phases[j] - self.phases[i])) * r_ij / distance
                    
                    # 2. Natural repulsion that increases as distance decreases
                    # This automatically creates stronger repulsion at short distances
                    # and very weak repulsion at long distances
                    repulsion_factor = self.repulsion_strength * np.exp(-distance / self.repulsion_scale)
                    repulsion = -repulsion_factor * r_ij / (distance-0.2)
                    
                    # Combine attraction and repulsion
                    total_force = attraction + repulsion
                    
                    # Update position derivatives (x and y only, maintain z)
                    position_derivatives[i][0] += total_force[0]
                    position_derivatives[i][1] += total_force[1]
                    
                    # Calculate phase interaction (phase velocity)
                    phase_coupling = self.K * np.sin(self.phases[j] - self.phases[i]) / self.num_cfs
                    phase_derivatives[i] += phase_coupling
        
        # Apply maximum velocity constraint
        for i in range(self.num_cfs):
            # Limit horizontal velocity for smooth movement
            xy_vel = np.array([position_derivatives[i][0], position_derivatives[i][1]])
            speed = np.linalg.norm(xy_vel)
            if speed > self.max_vel:
                xy_vel = xy_vel * self.max_vel / speed
                position_derivatives[i][0] = xy_vel[0]
                position_derivatives[i][1] = xy_vel[1]
            
            # Ensure we don't move beyond the allowed radius
            distance_from_center = np.linalg.norm(self.positions[i][:2] - np.mean(self.positions[:, :2], axis=0))
            if distance_from_center > self.radius:
                # Add a component that pushes back toward the center
                center_dir = np.mean(self.positions[:, :2], axis=0) - self.positions[i][:2]
                center_dir = center_dir / np.linalg.norm(center_dir)
                position_derivatives[i][0] += 0.2 * center_dir[0]
                position_derivatives[i][1] += 0.2 * center_dir[1]
        
        return position_derivatives, phase_derivatives

    def update_positions_from_lighthouse(self):
        """Update the positions using actual data from positioning system."""
        # In simulation mode, we don't do anything here since positions are synthetic
        if self.is_simulation:
            return
            
        # In hardware mode, try to get real position data if available
        try:
            for i, cf in enumerate(self.crazyflies):
                # Try to get position from status if available
                if hasattr(cf, 'status') and cf.status:
                    if 'pose' in cf.status:
                        pos = np.array([
                            cf.status['pose']['position']['x'],
                            cf.status['pose']['position']['y'],
                            cf.status['pose']['position']['z']
                        ])
                        self.positions[i] = pos
                    elif hasattr(cf, 'position') and callable(cf.position):
                        # Alternative method if available
                        try:
                            pos = cf.position()
                            if pos is not None and len(pos) == 3:
                                self.positions[i] = np.array(pos)
                        except:
                            pass
        except Exception as e:
            print(f"Warning: Could not update positions: {e}")
            # Continue with existing position data

    def start_continuous_position_updates(self):
        """Start a separate thread for continuous position updates."""
        if not self.continuous_position_update:
            return
            
        # Create and start position update thread
        self.is_running = True
        self.position_thread = threading.Thread(
            target=self.continuous_position_update_loop,
            daemon=True
        )
        self.position_thread.start()
        print(f"Started continuous position update thread at {self.continuous_update_rate}Hz")
    
    def stop_continuous_position_updates(self):
        """Stop the continuous position update thread."""
        self.is_running = False
        if self.position_thread and self.position_thread.is_alive():
            self.position_thread.join(timeout=1.0)
            print("Stopped continuous position update thread")
    
    def custom_sleep(self, duration):
        """Sleep for the specified duration without calling rclpy.spin_once().
        
        This is a replacement for timeHelper.sleep that avoids ROS executor conflicts.
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(0.001)  # Short sleep to prevent CPU hogging
    
    def continuous_position_update_loop(self):
        """Continuously update drone movements at the specified rate."""
        sleep_time = 1.0 / self.continuous_update_rate
        update_count = 0
        last_time = time.time()  # Use time.time() instead of timeHelper.time()
        
        while self.is_running and not self.timeHelper.isShutdown():
            loop_start = time.time()
            
            # Acquire lock to safely access position data
            with self.position_lock:
                # Update real positions from external systems if available
                if not self.is_simulation:
                    self.update_positions_from_lighthouse()
                
                # Use velocity control with cmdFullState
                for i, cf in enumerate(self.crazyflies):
                    # Get the velocity we calculated
                    vx = self.velocities[i][0]
                    vy = self.velocities[i][1]
                    vz = 0.0  # Maintain altitude
                    
                    # Get current position
                    current_pos = self.positions[i].copy()
                    
                    # Zero acceleration and angular velocity (except yaw if needed)
                    acc = [0.0, 0.0, 0.0]
                    yaw = 0.0  # Current yaw or desired yaw
                    omega = [0.0, 0.0, 0.0]  # No angular velocity
                    
                    # Send full state command with velocity
                    cf.cmdFullState(
                        current_pos,  # Current position
                        [vx, vy, vz],  # Desired velocity
                        acc,  # Zero acceleration
                        yaw,  # Current yaw
                        omega  # No angular velocity
                    )
            
            # Measure and report update frequency
            update_count += 1
            if update_count % 100 == 0:
                now = time.time()
                elapsed = now - last_time
                freq = 100 / elapsed if elapsed > 0 else 0
                print(f"Control command frequency: {freq:.1f}Hz")
                last_time = now
            
            # Calculate remaining time to sleep
            elapsed = time.time() - loop_start
            remaining = sleep_time - elapsed
            
            # Sleep for the appropriate time using custom sleep to avoid ROS spin conflicts
            if remaining > 0:
                self.custom_sleep(remaining)

    def update_target_positions(self, pos_derivatives):
        """Update target positions and velocities based on the Swarmalator dynamics."""
        with self.position_lock:
            for i in range(self.num_cfs):
                # Store velocity for velocity-based control
                self.velocities[i][0] = pos_derivatives[i][0]  # x velocity
                self.velocities[i][1] = pos_derivatives[i][1]  # y velocity
                self.velocities[i][2] = 0.0  # z velocity (maintain height)
                
                # Update internal position record based on derivatives (for visualization)
                # This helps the dynamics even if actual drone position varies
                self.positions[i] = self.positions[i] + np.array([
                    pos_derivatives[i][0] * self.dt,
                    pos_derivatives[i][1] * self.dt,
                    0  # Keep height constant
                ])
                
                # Ensure z-coordinate remains constant
                self.positions[i][2] = self.height

    def run(self, duration=60):
        """Run the Swarmalator simulation for the specified duration."""
        try:
            # Initialize drone positions
            self.initialize_positions()
            
            # Initialize target positions from current positions
            with self.position_lock:
                self.target_positions = self.positions.copy()
            
            # Enable LED decks - only in hardware mode
            if not self.is_simulation:
                for cf in self.crazyflies:
                    try:
                        cf.setParam("ring", "effect", 7)  # Solid color mode
                    except Exception as e:
                        print(f"Warning: Could not enable LED deck: {e}")
                        # Mark this drone so we don't keep trying to set LEDs
                        cf._led_error_shown = True
            
            # Start continuous position update thread if enabled
            if self.continuous_position_update:
                self.start_continuous_position_updates()
            
            # Main control loop
            start_time = self.timeHelper.time()
            last_position_update = 0
            update_count = 0
            
            print(f"Running main control loop at {1/self.dt:.1f}Hz")
            
            while self.timeHelper.time() - start_time < duration and not self.timeHelper.isShutdown():
                loop_start = time.time()  # Use time.time() for consistent timing
                
                # Update counter for position updates
                self.position_update_counter += 1
                
                # Try to update positions based on the update ratio
                if self.position_update_counter >= self.position_update_ratio:
                    self.position_update_counter = 0
                    
                    # Only update from lighthouse in the main thread if continuous updates are disabled
                    if not self.continuous_position_update:
                        with self.position_lock:
                            self.update_positions_from_lighthouse()
                    
                    update_count += 1
                    
                    # Calculate and print update frequency every 50 updates
                    if update_count % 50 == 0:
                        current_time = self.timeHelper.time()
                        if last_position_update > 0:
                            freq = 50 / (current_time - last_position_update)
                            print(f"Main loop frequency: {freq:.1f}Hz")
                        last_position_update = current_time
                
                # Compute Swarmalator dynamics
                pos_derivatives, phase_derivatives = self.compute_swarmalator_dynamics()
                
                # Update phases
                self.phases += phase_derivatives * self.dt
                
                # Normalize phases to [0, 2π)
                self.phases = self.phases % (2 * np.pi)
                
                # Update LEDs based on phases (less frequently to reduce CPU load)
                if self.position_update_counter == 0:
                    self.update_leds()
                
                # Update target positions based on current dynamics
                self.update_target_positions(pos_derivatives)
                
                # If continuous updates are disabled, send commands here
                if not self.continuous_position_update:
                    with self.position_lock:
                        # Use velocity control with cmdFullState
                        for i, cf in enumerate(self.crazyflies):
                            # Get the velocity we calculated
                            vx = self.velocities[i][0]
                            vy = self.velocities[i][1]
                            vz = 0.0  # Maintain altitude
                            
                            # Get current position
                            current_pos = self.positions[i].copy()
                            
                            # Zero acceleration and angular velocity (except yaw if needed)
                            acc = [0.0, 0.0, 0.0]
                            yaw = 0.0  # Current yaw or desired yaw
                            omega = [0.0, 0.0, 0.0]  # No angular velocity
                            
                            # Send full state command with velocity
                            cf.cmdFullState(
                                current_pos,  # Current position
                                [vx, vy, vz],  # Desired velocity
                                acc,  # Zero acceleration
                                yaw,  # Current yaw
                                omega  # No angular velocity
                            )
                
                # Calculate actual loop execution time
                execution_time = time.time() - loop_start
                if execution_time > self.dt and self.position_update_counter % 100 == 1:
                    print(f"Warning: Loop execution time ({execution_time:.4f}s) exceeds time step ({self.dt:.4f}s)")
                
                # Sleep to maintain control rate using ROS spin lock
                with self.ros_spin_lock:
                    self.timeHelper.sleepForRate(1/self.dt)
                
            # Stop continuous position updates
            self.stop_continuous_position_updates()
            
            # Land all drones
            self.swarm.allcfs.land(targetHeight=0.06, duration=3.0)
            self.timeHelper.sleep(3.5)
            
        except Exception as e:
            print(f"Error in Swarmalator control: {e}")
            import traceback
            traceback.print_exc()
            # Emergency land if there's an error
            self.swarm.allcfs.emergency()
        finally:
            # Make sure the continuous update thread is stopped
            self.stop_continuous_position_updates()


def main():
    """Main function to run the Swarmalator simulation."""
    try:
        # Create and run the Swarmalator controller
        controller = SwarmalatorController()
        controller.run(duration=120)  # Run for 2 minutes
    except KeyboardInterrupt:
        print("Simulation stopped by user")
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
