import re
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as m
from scipy.spatial.transform import Rotation as Rota
from numpy.linalg import norm 

##############################################################################
##############################################################################
##############################################################################
def lecture_quat(link):
    all_lines = []
    with open(link ,'r') as fh:
        all_lines = fh.readlines()    
    coord = []
    for i in all_lines:
        temp =  i.split()
        coord.append( [float(temp[0]),float(temp[1]),float(temp[2]),float(temp[3])] )            
    return coord

def lecture_coord(link):
    all_lines = []
    with open(link ,'r') as fh:
        all_lines = fh.readlines()    
    coord = []
    for i in all_lines:
        temp =  i.split()
        coord.append( [float(temp[0]),float(temp[1]),float(temp[2])] )            
    return coord

def lecture_pos(link):
    all_lines = []
    with open(link ,'r') as fh:
        all_lines = fh.readlines()    
    coord_av = []
    coord_ar = []
    for i in all_lines:
        temp =  i.split()
        if temp[0][0] == 'F':
            coord_av.append( [float(temp[1]),float(temp[2]),float(temp[3])] )
        # if temp[0][0] == 'B':
        #     coord_ar.append( [float(temp[1]),float(temp[2]),float(temp[3])] )              
    return coord_av, coord_ar

def readOPKFile(filename):
    # Exported from Agisoft format OPK
    # Cameras (105)
    # PhotoID, X, Y, Z, Omega, Phi, Kappa, r11, r12, r13, r21, r22, r23, r31, r32, r33
    out = {}
    with open(filename) as f:
        for l in f:
            if l.startswith("#"):
                continue
            l = l.split()
            x,y,z = float(l[1]),float(l[2]),float(l[3])
            S = np.asarray([x,y,z])
            r11, r12, r13, r21, r22, r23, r31, r32, r33 = [float(l[i]) for i in range(7,16)]
            R = np.array([
                [r11, r12, r13],
                [r21, r22, r23],
                [r31, r32, r33],
            ])
            out[l[0]] = [S,R]
    return out

def lecture_coord_cam(cam, link):
    all_lines = []
    with open(link ,'r') as fh:
        all_lines = fh.readlines()    
    coord = []
    c = 0
    for i in all_lines:
        temp =  i.split()
        if str(temp[0]) == cam :
            index = c
            coord.append( [float(temp[1]),float(temp[2]),float(temp[3])] )  
        c += 1
    return coord[0], index

def lecture_matrix_cam(cam, link):
    all_lines = []
    with open(link ,'r') as fh:
        all_lines = fh.readlines()    
    for i in all_lines:
        temp =  i.split()
        if str(temp[0]) == cam :
            R_temp = np.array([[float(temp[1]),float(temp[2]),float(temp[3])],
                             [float(temp[4]),float(temp[5]),float(temp[6])],
                             [float(temp[7]),float(temp[8]),float(temp[9])]])
            return R_temp
    return


def lecture_obs(lien):
    all_lines = []
    with open(lien ,'r') as fh:
        all_lines = fh.readlines()    
    obs_valeur = []
    for i in all_lines:
        temp =  i.split()
        obs_valeur.append({'X':float(temp[2]),'Y': float(temp[3]),'Z': float(temp[4]),'x': float(temp[0]),'y': float(temp[1]) })  
    return obs_valeur




def lecture_marker(lien):
    all_lines = []
    with open(lien ,'r') as fh:
        all_lines = fh.readlines()    
    obs_valeur = {}
    c = 0
    for i in all_lines:
        temp =  i.split()
        v = int(temp[0])
        w = {str(c):{'Id_marker':c,'Id_cam':v, 'X': float(temp[1]),'Y': float(temp[2]),'Z': float(temp[3]),'x': float(temp[4]), 'y': float(temp[5])}}
        obs_valeur[v] = [w] if v not in obs_valeur.keys() else obs_valeur[v] + [w]
        c += 1 
    return obs_valeur



