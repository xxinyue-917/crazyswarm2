import math
import numpy as np
import time
import threading
import sys
import select
import cvxpy as cp
from crazyflie_py import Crazyswarm
#note:
# set up simulation 
# figure out a way to incorporate the timestamp changes 



class Swarmalator:
    def __init__(self):
        # Initialization variables
        self.botRad = 0.05
        self.numBots = len(self.crazyflies.crazyflies)
        self.plotXLimit = 2.15
        self.plotYLimit = 3.25
        self.plotZLimit = 1
        self.dt = 0.1
        self.A = 1
        self.B = 1
        self.J = 3
        self.K = 1

        # Initialization of variables for each bot
        self.positions = np.zeros((self.numBots, 3))
        self.phases = np.zeros(self.numBots)
        # Store the position and phase information
        self.dPos = np.zeros((self.numBots, 3))
        self.dPhase = np.zeros(self.numBots)
        self.final_vel = np.zeros((self.numBots, 3))
        # Parameters for the swarmalator
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.crazyflies = self.swarm.allcfs
        

        # Initialization of obstacles
        self.numObstacles = 1
        # x, y, z, radius
        self.obstacles = np.zeros((self.numObstacles, 4))
        self.obstacles[0] = [1.5, -1, 0.7, 0.19]

        # Initialize trajectory 
        self.final_time_step = 2000
        self.timestep = 0
        self.traj = np.zeros((self.final_time_step, 3))
        self.trajx_start = 0.25
        self.trajy_start = -0.25
        self.trajx_end = 3
        self.trajy_end = -2

        # Parameters for CLF-CBF
        self.eps = 10
        self.k = 0.5

        # Shape of the ellipsoid
        self.a = 2.0 * self.botRad # x–axis radius
        self.b = 2.0 * self.botRad # y–axis radius
        self.c = 6.0 * self.botRad # z–axis radius

        # Initialize the positions and phases
        self.initialize_positions()
        self.initialize_phases()
        self.initialize_trajectory()
        
        # Flag to control execution
        self.running = False

        # Unsure variables
        self.height = 0.5
        self.minVel = -0.5
        self.maxVel = 0.5

    def initialize_trajectory(self):
        for t in range(self.final_time_step):
            if t < 300:
                # collect at (0.25, -0.25, 0.7)
                self.traj[t] = [self.trajx_start, self.trajy_start, 0.7]
            elif t < 1700:
                # interpolate from (0.25, -0.25) to (3, -2) with z = 0.7
                ratio = (t - 300) / (1700 - 300)  # (t-300)/1400
                pos_x = (self.trajx_end - self.trajx_start) * ratio + self.trajx_start
                pos_y = (self.trajy_end - self.trajy_start) * ratio + self.trajy_start
                self.traj[t] = [pos_x, pos_y, 0.7]
            else:
                # stay at (12, 12, 1)
                self.traj[t] = [12, 12, 1]

    def initialize_positions(self):
        self.crazyflies.takeoff(targetHeight=self.height, duration=1.0+self.height)
        self.timeHelper.sleep(1.5+self.height)
        for i, cf in enumerate(self.crazyflies.crazyflies):
            pos = np.array(cf.initialPosition) + np.array([0, 0, self.height])
            cf.goTo(pos, 0, 1.0)
            self.positions[i] = pos

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
        self.timeHelper.sleep(3)  # Wait for landing to complete
        print("All Crazyflies landed safely")
    
    def swarmalator_model(self):
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
                    dz = self.positions[j][2] - self.positions[i][2]
                    dist = np.sqrt(dx**2 + dy**2 + dz**2)
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
                    self.dPos[i][2] += F_total * dz

                    # Compute the phase attraction
                    self.dPhase[i] += self.K * np.sin(self.phases[j] - self.phases[i])/dist

    def update_positions(self):
        # Update the positions and phases
        for i in range(self.numBots):
            # Update the position
            # self.dPos[i][0] = max(self.minVel, min(self.dPos[i][0], self.maxVel))
            # self.dPos[i][1] = max(self.minVel, min(self.dPos[i][1], self.maxVel))
            # self.dPos[i][2] = max(self.minVel, min(self.dPos[i][2], self.maxVel))
            self.final_vel[i][0] = max(self.minVel, min(self.final_vel[i][0], self.maxVel))
            self.final_vel[i][1] = max(self.minVel, min(self.final_vel[i][1], self.maxVel))
            self.final_vel[i][2] = max(self.minVel, min(self.final_vel[i][2], self.maxVel))
            # print("dPos of Drone: ", i, " is: ", self.dPos[i])
            self.positions[i][0] += self.final_vel[i][0] * self.dt
            self.positions[i][1] += self.final_vel[i][1] * self.dt
            self.positions[i][2] += self.final_vel[i][2] * self.dt
            # print("Position of Drone: ", i, " is: ", self.positions[i])

            # Regularize and update the phase
            self.phases[i] += self.dPhase[i] * self.dt
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

    def clf_cbf(self):

        px = self.traj[self.timestep][0]
        py = self.traj[self.timestep][1]
        pz = self.traj[self.timestep][2]
        for i in range(self.numBots):
            # Calculate the collective center
            mean_center = np.mean(self.positions, axis=0)
            mean_centerX = mean_center[0]
            mean_centerY = mean_center[1]
            mean_centerZ = mean_center[2]

            centerX = self.positions[i, 0]
            centerY = self.positions[i, 1]
            centerZ = self.positions[i, 2]
            dX = self.dPos[i][0]
            dY = self.dPos[i][1]
            dZ = self.dPos[i][2]

            # Lyapunov function for target point
            V = (centerX - px)**2 + (centerY - py)**2 + (centerZ - pz)**2

            # Time derivative of V
            LfV = 2 * (centerX - px) * dX + 2 * (centerY - py) * dY + 2 * (centerZ - pz) * dZ
            LgV_u1 = 2 * (centerX - px)
            LgV_u2 = 2 * (centerY - py)
            LgV_u3 = 2 * (centerZ - pz)
            A_clf = [LgV_u1, LgV_u2, LgV_u3, -1]   # -1 is for relaxation variable
            b_clf = -LfV - self.eps * V

            # Control Barrier Function
            A_cbf_obs = []
            b_cbf_obs = []

            for j in self.numObstacles:
                ox = self.obstacles[j][0]
                oy = self.obstacles[j][1]
                oz = self.obstacles[j][2]
                r = self.obstacles[j][3]
                b_j = (centerX - ox)**2 + (centerY - oy)**2 + (centerZ - oz)**2 - (r + self.botRad)**2
                Lgb_u1_j = 2*(centerX - ox)
                Lgb_u2_j = 2*(centerY - oy)
                Lgb_u3_j = 2*(centerZ - oz)
                Lfb_j = Lgb_u1_j*dX + Lgb_u2_j*dY + Lgb_u3_j*dZ
                A_temp = [-Lgb_u1_j, -Lgb_u2_j, -Lgb_u3_j, 0]
                b_temp = Lfb_j + self.k*b_j

                # Add this constraint to all the constraints
                A_cbf_obs.append(A_temp)
                b_cbf_obs.append(b_temp) 
                
            # Construct a constraint for all other agents
            A_cbf_all = []
            b_cbf_all = []

            for j in self.numBots:
                if j != i:
                    # Location of each agent 
                    obsX = self.positions[j][0]
                    obsY = self.positions[j][1]
                    obsZ = self.positions[j][2]

                    # If treat each agent as an ellipsoid
                    b_ij = ((centerX - obsX) / self.a)**2 + ((centerY - obsY) / self.b)**2 + ((centerZ - obsZ) / self.c)**2
                    # Lie derivative along each control input u1,u2,u3:
                    Lgb_ij_u1 = 2*(centerX - obsX)/self.a**2
                    Lgb_ij_u2 = 2*(centerY - obsY)/self.b**2
                    Lgb_ij_u3 = 2*(centerZ - obsZ)/self.c**2
                    # Lie derivative along fx (drift dX,dY,dZ):
                    Lfb_ij = Lgb_ij_u1*dX + Lgb_ij_u2*dY + Lgb_ij_u3*dZ
                    # Construct Constraint： -Lgb_ij * [u1; u2; u3] + Lfb_ij + k*b_ij >= 0
                    # Constraint： A_cbf * [u1; u2; relaxL; relaxB] <= b_cbf
                    A_temp = [-Lgb_ij_u1, -Lgb_ij_u2, -Lgb_ij_u3, 0]
                    b_temp = Lfb_ij + self.k * b_ij

                    # Add this constraint to all the constraints
                    A_cbf_all.append(A_temp)
                    b_cbf_all.append(b_temp)
            
            # Quadratic programming to find control
            v_max = 10
            A_1 = np.vstack([
                A_clf,
                A_cbf_obs,
                A_cbf_all,
                [ 1,  0,  0,  0],
                [-1,  0,  0,  0],
                [ 0,  1,  0,  0],
                [ 0, -1,  0,  0],
                [ 0,  0,  1,  0],
                [ 0,  0, -1,  0]
            ])
            b_1 = np.vstack([
                b_clf,
                b_cbf_obs,
                b_cbf_all,
                np.full((6, 1), v_max)
            ])
            H = np.array([
                [2,   0,   0,    0],
                [0,   2,   0,    0],
                [0,   0,   2,    0],
                [0,   0,   0, 0.02]
            ])
            F = np.zeros((4, 1))  # Same as [0; 0; 0; 0]
            x = cp.Variable((4, 1))
            # Define the quadratic program
            objective = cp.Minimize(0.5 * cp.quad_form(x, H) + F.T @ x)
            constraints = [A_1 @ x <= b_1]
            # Solve the problem
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP, verbose=False)
            ctrl = x.value
            ctrlX = None
            ctrlY = None
            ctrlZ = None

            # Update control velocities
            if ctrl is not None:
                ctrlX = ctrl[0, 0]
                ctrlY = ctrl[1, 0]
                ctrlZ = ctrl[2, 0]
                relaxL = ctrl[3, 0]
            else:
                ctrlX = 0
                ctrlY = 0
                ctrlZ = 0
                self.dPos[i][0] = 0
                self.dPos[i][1] = 0
                self.dPos[i][2] = 0
                print('Unsolvable!')

            # Calculate the final linear velocity and angular velocity
            self.final_vel[i][0] = self.dPos[i][0] + ctrlX
            self.final_vel[i][1] = self.dPos[i][1] + ctrlY
            self.final_vel[i][2] = self.dPos[i][2] + ctrlZ

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
                self.swarmalator_model()
                self.clf_cbf()
                self.update_positions()
                for i in self.numBots:
                    self.crazyflies.crazyflies[i].goTo(self.positions[i], 0, 0.1)

                self.timeHelper.sleep(0.1)
            
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