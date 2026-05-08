import re
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as m
from scipy.spatial.transform import Rotation as Rota
from numpy.linalg import norm 

def quat_to_euler(quat):
    angle_deg = []
    for i in range(len(quat)):
        kpo = Rotation.from_quat(quat[i]).as_euler ( 'xyz', degrees=True)
        angle_deg.append(kpo)
        #cam.reference.rotation = ms.Vector(kpo[::-1])
    return angle_deg

def sph2EN(projection,lon_deg,lat_deg,R):
    lon = lon_deg*np.pi/180.0
    lat = lat_deg*np.pi/180.0
    if projection == 'plate_carree':
        E = R*lon
        N = R*lat
    return E,N

def sph2cart(lon_deg,lat_deg,r):
    lon_rad = lon_deg*np.pi/180.0
    lat_rad = lat_deg*np.pi/180.0 
    x = r*np.cos(lat_rad)*np.cos(lon_rad)
    y = r*np.cos(lat_rad)*np.sin(lon_rad)
    z = r*np.sin(lat_rad)
    return x,y,z


def matrice_rot_cardan_xyz(alpha_deg, beta_deg, gamma_deg ):
    r = Rotation.from_euler('xyz',[alpha_deg, beta_deg, gamma_deg], degrees=True).as_matrix()
    return r
