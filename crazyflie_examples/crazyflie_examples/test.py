from crazyflie_py import Crazyswarm
import numpy as np

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    start_time = timeHelper.time()
    dt = 0.1
    duration = 30.0
    drone1 = swarm.allcfs.crazyflies[0]

    drone1.takeoff(targetHeight=0.3, duration=2.5)
    timeHelper.sleep(2.5)

    # Test the cmdFullState function
    pos = np.array(drone1.initialPosition) + np.array([0, 0, 0.3])
    vel1 = np.array([0.5, 0.0, 0.0])
    vel2 = np.array([-0.5, 0.0, 0.0])
    yaw = 0.0
    try:
        while timeHelper.time() - start_time < duration:
            # drone1.cmdFullState(
            #     pos + vel1 * dt,
            #     vel1,
            #     np.array([0.0, 0.0, 0.0]),
            #     yaw,
            #     np.array([0.0, 0.0, 0.0])
            # )
            drone1.goTo(pos + vel1 * dt, 0, 0.1)
            timeHelper.sleep(dt)
            # drone1.cmdFullState(
            #     pos + vel2 * dt,
            #     vel2,
            #     np.array([0.0, 0.0, 0.0]),
            #     yaw,
            #     np.array([0.0, 0.0, 0.0])
            # )
            drone1.goTo(pos + vel2 * dt, 0, 0.1)
            timeHelper.sleep(dt)
    except KeyboardInterrupt:
        drone1.land(targetHeight=0.04, duration=2.5)
        timeHelper.sleep(2.5)
        print("Test interrupted by user")

    finally:
        drone1.land(targetHeight=0.04, duration=2.5)
        timeHelper.sleep(2.5)

if __name__ == "__main__":
    main()