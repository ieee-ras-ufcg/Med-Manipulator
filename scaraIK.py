from coppeliasim_zmqremoteapi_client import *
import math
import time

client = RemoteAPIClient()
sim = client.require('sim')


L1, L2 = 0.71663, 0.60

mtb = sim.getObject("/MTB")
joint1 = sim.getObject("/MTB/axis")
joint2 = sim.getObject("/MTB/link/axis")
joint3 = sim.getObject("/MTB/link/axis/link/axis")
finalScara = sim.getObject("/MTB/suctionPad/LoopClosureDummy2")
rec = sim.getObject("/Rectangle")


def inverse_kinematics(x, y, z):
    dist = math.hypot(x, y)
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)
    if dist > max_reach or dist < min_reach:
        return None
    
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1), -1)
    theta2 = math.acos(cos_theta2)
    den = L1 + L2 * math.cos(theta2)
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), den)
    return theta1, theta2, z


def move_robot(theta1, theta2, theta3):
    sim.setJointTargetPosition(joint1, theta1)
    time.sleep(0.02)
    sim.setJointTargetPosition(joint2, theta2)
    time.sleep(0.02)
    sim.setJointTargetPosition(joint3, theta3)
    time.sleep(0.02)


def follow_rectangle():
    while True:    
        target_pos = sim.getObjectPosition(rec, mtb) 

        angles = inverse_kinematics(-target_pos[0], -target_pos[1], target_pos[2])

        if angles:
            theta1, theta2, theta3 = angles
            move_robot(theta1, theta2, theta3)



sim.startSimulation()
follow_rectangle()
time.sleep(0.02)

sim.stopSimulation()
