from coppeliasim_zmqremoteapi_client import *
import math
import numpy as np
import time

client = RemoteAPIClient()
sim = client.require('sim')

mtb = sim.getObject("/MTB")
handles = [
    sim.getObject("/MTB/axis"),
    sim.getObject("/MTB/link/axis"),
    sim.getObject("/MTB/link/axis/link/axis"),
    sim.getObject("/MTB/link/axis/link/axis/axis")
]
suctionPad = sim.getObject("/MTB/suctionPad")

# DH parameters
a1 = 0.71663  # primeiro elo
a2 = 0.6      # segundo elo
d1 = 0.1      # altura da base
d4 = 0.05     # efetuador

default_pose = [-90*math.pi/180,3*math.pi/180, -120*math.pi/180,-35*math.pi/180]
final_position = [math.pi/180, math.pi/6, math.pi/2, math.pi/4]

vel = 120 * math.pi/180 
accel = 40 * math.pi/180
jerk = 80 * math.pi/180
maxVel = [vel*math.pi/180] * 5
maxAccel = [accel*math.pi/180] * 5
maxJerk = [jerk*math.pi/180] * 5

def moveToConfig(handles, default_pose, maxVel, maxAccel, maxJerk):
    params = {
        'joints': handles,
        'targetPos': default_pose,
        'vel': maxVel,
        'accel': maxAccel,
        'jerk': maxJerk,
    }
    sim.moveToConfig(params)

def forwardKinematics(theta1, theta2, d3, theta4):
    A1 = np.array([
        [math.cos(theta1), -math.sin(theta1), 0, a1 * math.cos(theta1)],
        [math.sin(theta1), math.cos(theta1), 0, a1 * math.sin(theta1)],
        [0, 0, 1, d1],
        [0, 0, 0, 1]
    ])
    
    A2 = np.array([
        [math.cos(theta2), -math.sin(theta2), 0, a2 * math.cos(theta2)],
        [math.sin(theta2), math.cos(theta2), 0, a2 * math.sin(theta2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    A3 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d3],
        [0, 0, 0, 1]
    ])
    
    A4 = np.array([
        [math.cos(theta4), -math.sin(theta4), 0, 0],
        [math.sin(theta4), math.cos(theta4), 0, 0],
        [0, 0, 1, d4],
        [0, 0, 0, 1]
    ])
    
    return A1 @ A2 @ A3 @ A4

sim.startSimulation()

t_final = forwardKinematics(*final_position)
final_pos = t_final[0:3, 3]
print(f"Expected end effector position: {final_pos}")

moveToConfig(handles, final_position, maxVel, maxAccel, maxJerk)


actual_pos = sim.getObjectPosition(suctionPad, -1)
print(f"Actual position coppelia: {actual_pos}")
time.sleep(5)  
sim.stopSimulation()  
