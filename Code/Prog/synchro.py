import re
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as m
from scipy.spatial.transform import Rotation as Rota
from numpy.linalg import norm 

def synchro_gps_fps(pos, euler, grav, magn): # , gyro
    euler_sync = []
    grav_sync = [] 
    # gyro_sync = []
    magn_sync = []
    for i in range(int(len(pos))):
        euler_sync.append([euler[int(i*10)][0], euler[int(i*10)][1], euler[int(i*10)][2]])
        grav_sync.append(grav[int(i*10)])
        magn_sync.append(magn[int(i*10)])
        # gyro_sync.append(gyro[int(i*800/2.4)]) #normalement 200 d'après la doc
    return pos, euler_sync, grav_sync, magn_sync #gyro_sync