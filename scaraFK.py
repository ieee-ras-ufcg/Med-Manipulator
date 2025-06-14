
from coppeliasim_zmqremoteapi_client import *
import math
import numpy as np
import time

client = RemoteAPIClient()
sim = client.require('sim')

mtb = sim.getObject("/MTB")
joint1 = sim.getObject("/MTB/axis")
joint2 = sim.getObject("/MTB/link/axis")
joint3 = sim.getObject("/MTB/link/axis/link/axis")
joint4 = sim.getObject("/MTB/link/axis/link/axis/axis")
suctionPad = sim.getObject("/MTB/suctionPad")

#dh
a1 = 0.467 #primeiro elo
a2 = 0.4005  #segundo elo
d1 = 0.152 #altura da base
d4 = 0.05 #efetuador


final_position = [math.pi/2, math.pi/4, -0.20, math.pi/2]

params = {
    'joints': [joint1, joint2, joint3, joint4],  
    'targetPos': final_position,               
    'maxVel': [3, 3, 2, 3],           
    'maxAccel': [3, 3, 3, 3],        
    'maxJerk': [2, 2, 1, 2],                      
}
movement_time=5

def forwardKinematics(theta1, theta2, d3, theta4):
    A1 = np.array([
        [math.cos(theta1), -math.sin(theta1), 0, a1 * math.cos(theta1)],
        [math.sin(theta1),  math.cos(theta1), 0, a1 * math.sin(theta1)],
        [0,                0,                1, d1],
        [0,                0,                0, 1]
    ])

    A2 = np.array([
        [math.cos(theta2), -math.sin(theta2), 0, a2 * math.cos(theta2)],
        [math.sin(theta2),  math.cos(theta2), 0, a2 * math.sin(theta2)],
        [0,                0,                1, 0],
        [0,                0,                0, 1]
    ])

    A3 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d3],  
        [0, 0, 0, 1]
    ])

    A4 = np.array([
        [math.cos(theta4), -math.sin(theta4), 0, 0],
        [math.sin(theta4),  math.cos(theta4), 0, 0],
        [0,                0,                1, d4],
        [0,                0,                0, 1]
    ])
 
    T = (A1 @ A2 @ A3 @ A4)

    return T


t_final = forwardKinematics(*final_position)
final_pos = t_final[0:3,3]

sim.startSimulation()
sim.moveToConfig(params)
time.sleep(5)
print(f"posição final do efetuador pelo codigo: {final_pos}")
print(f"posição final do efetuador pelo coppelia: {sim.getObjectPosition(suctionPad, mtb)}")
time.sleep(60)
