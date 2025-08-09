# --- Bibliotecas e Chamada da API ---
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np
import time

# --- Conexão e Definições Iniciais ---
client = RemoteAPIClient()
sim = client.require('sim')

# --- Parâmetros do Braço ---
L1 = 0.467
L2 = 0.4005

# --- Handles dos Objetos ---
joint1 = sim.getObject("/MTB/axis")
joint2 = sim.getObject("/MTB/link/axis")
cube = sim.getObject("/MTB/Rectangle")

joint_handles_list = [joint1, joint2]

# --- Cinemática Inversa (IK) ---
def inverse_kinematics(x, y):
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1), -1)
    theta2 = math.acos(cos_theta2)
    
    den = L1 + L2 * math.cos(theta2)
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), den)
    
    return theta1, theta2
   

# --- Movimento do Braço ---
def move_to_target(theta1, theta2):
    sim.setJointTargetPosition(joint1, theta1)
    sim.setJointTargetPosition(joint2, theta2)

def cube_following():
    while True:
        cube_position = sim.getObjectPosition(cube, -1)
        angles = inverse_kinematics(cube_position[0], cube_position[1])
        theta1,theta2 = angles
        move_to_target(theta1,theta2)
sim.startSimulation()
cube_following()
time.sleep(60)
sim.stopSimulation()
