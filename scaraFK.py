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
a1 = 0.71663 #primeiro elo
a2 = 0.6 #segundo elo
d1 = 0.1 #altura da base
d4 = 0.05 #efetuador

time_steps = np.linspace(0,5,500)
dt = time_steps[1] - time_steps[0]

theta1_vals = np.linspace(0, math.pi/2, len(time_steps))
theta2_vals = np.linspace(0, math.pi/4, len(time_steps))
d3_vals = np.linspace(0, 0.15, len(time_steps))
theta4_vals = np.linspace(0, math.pi/6, len(time_steps))

def forwardKinematics(theta1, theta2, d3, theta4):
    a1_array = np.array([
        [math.cos(theta1), -math.sin(theta1), 0, a1*math.cos(theta1)],
        [math.sin(theta1), math.cos(theta1), 0, a1*math.sin(theta1)],
        [0, 0, 1, d1],
        [0, 0, 0, 1]
    ])
    a2_array = np.array([
        [math.cos(theta2), -math.sin(theta2), 0, a2*math.cos(theta2)],
        [math.sin(theta2), math.cos(theta2), 0, a2*math.sin(theta2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    a3_array = np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,d3],
        [0,0,0,1]
    ])
    a4_array = np.array([
        [math.cos(theta4), -math.sin(theta4), 0, 0],
        [math.sin(theta4),  math.cos(theta4), 0,0 ],
        [0,           0,           1, d4],
        [0,           0,           0, 1]
    ])

    return (a4_array @ a3_array @ a2_array @ a1_array)


positions = []

for t1,t2,t3,t4 in zip(theta1_vals,theta2_vals,d3_vals,theta4_vals):
    t = forwardKinematics(t1,t2,t3,t4)
    pos = t[0:3,3]
    positions.append(pos)
positions = np.array(positions)

def jerk_simple(trajectory, dt):
    velocity = np.gradient(trajectory, dt)
    acceleration = np.gradient(velocity, dt)
    jerk = np.gradient(acceleration, dt)
    return jerk

jerk_values = jerk_simple(positions,dt)
jerk_mag = np.linalg.norm(jerk_values)

final_pos = positions[-1]

sim.startSimulation()
for t1,t2,t3,t4 in zip(theta1_vals,theta2_vals,d3_vals,theta4_vals):
    sim.setJointPosition(joint1, t1)
    sim.setJointPosition(joint2, t2)
    sim.setJointPosition(joint3, t3)
    sim.setJointPosition(joint4, t4)
    time.sleep(dt)

print(f"posição final do efetuador pelo codigo: {final_pos}")
print(f"posição final do efetuador pelo coppelia: {sim.getObjectPosition(suctionPad)}")
time.sleep(60)
