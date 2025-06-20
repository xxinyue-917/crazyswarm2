"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm
import time
import threading
import rclpy
from crazyflie_examples.tracking import PositionTracker


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    # tracker = PositionTracker('tracker')


    # start_time = time.time()
    # while True:
    #     # Get all positions
    #             # Exit loop after 30 seconds
    #     if time.time() - start_time > 30:
    #         break
    #     frames = tracker.get_tracked_drones()
    #     # numBots = len(swarm.crazyflies.crazyflies)
    #     print(f"Tracked frames:{frames}")
        
    #     # Get specific position
    #     for frame in frames:
    #         pos = tracker.get_drone_position(frame)
    #         if pos:
    #             print(f"Position of {frame}: {pos}")   
    #         else:
    #             print("untracked")   
        
    #     time.sleep(1)
    
    # tracker.shutdown()
    while True:
        # for crazyfly in swarm.allcfs.crazyflies:
        #     # rclpy.spin_once(swarm.allcfs, )
        #     rclpy.spin_once(swarm.allcfs, timeout_sec=0.01)
        #     print(swarm.allcfs.position())
        #     time.sleep(1)
        rclpy.spin_once(swarm.track, timeout_sec=0.01)
        pos = swarm.track.get_position()
        if pos:
            x, y, z = pos
            print(f"Drone position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            # print(swarm.allcfs.crazyfliesByName)
        time.sleep(0.1)

    # timeHelper.sleep(TAKEOFF_DURATION)
    # for crazyfly in swarm.allcfs.crazyflies:
    #     crazyfly.land(targetHeight=0.04, duration=2.5)
    
    # timeHelper.sleep(TAKEOFF_DURATION) 

    # cf1 = swarm.allcfs.crazyflies[0]
    # # cf2 = swarm.allcfs.crazyflies[1]

    # cf1.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    # timeHelper.sleep(TAKEOFF_DURATION)
    # # cf2.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    # # timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    # cf1.land(targetHeight=0.04, duration=2.5)
    # timeHelper.sleep(TAKEOFF_DURATION) 
    # cf2.land(targetHeight=0.04, duration=2.5)
    # timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == '__main__':
    main()
