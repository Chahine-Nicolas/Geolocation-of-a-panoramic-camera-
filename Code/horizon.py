import numpy as np
# import matplotlib.pyplot as plt
import open3d.visualization.rendering as rendering
# import open3d as o3d
from scipy import ndimage
# from numpy.linalg import norm 
# from scipy.spatial.transform import Rotation as Rota
# from dtw import *
# import cv2
# import math

# import reframe as ref
# import eq_colinea as col
# import lecture as lec

# import tempfile
# import os

###############################################################################
###############################################################################

#Cherche la ligne d'horizon dans une image
def find_horizon(img):
    h, w = img.shape
    iimg, jimg = [], []
    for j in range(w):
        for i in range(h-1):
            if img[h-i-1][j] == 1 and img[h-i][j]!=1:
                if len(iimg) > j:
                    iimg[-1] = float(h-i)
                    jimg[-1] = float(j)
                else:
                    iimg.append(float(h-i))
                    jimg.append(float(j))
    return iimg, jimg


###############################################################################
#Cherche la ligne d'horizon dans un mesh
def create_horizon(mesh, center, up, FOV, S):
    material = rendering.MaterialRecord()
    render = rendering.OffscreenRenderer(2720, 2720)
    render.scene.add_geometry('mesh',mesh, material)
    render.setup_camera(FOV, S+center, S, up)
    depth = render.render_to_depth_image(z_in_view_space=True)
    d = np.array(depth)
    d[d != np.inf ] = 255
    d[d == np.inf ] = 0
    depthmap_op = np.array(ndimage.binary_opening( d, structure=np.ones((3,3))).astype(int))
    depthmap = np.array(depth)
    h, w = depthmap.shape
    imesh,  jmesh, dmesh = [], [], []
    for j in range(w):
        for i in range(h-1):
            if(depthmap_op[h-i-2][j] == 0 and depthmap_op[h-i-1][j] != 0):
                if j in jmesh:
                    imesh[jmesh.index(j)] = float(h-i-1)
                    jmesh[jmesh.index(j)] = float(j)
                    dmesh[jmesh.index(j)] = depthmap[h-i-1][j]
                else:
                    imesh.append(float(h-i-1))
                    jmesh.append(float(j))
                    dmesh.append(depthmap[h-i-1][j])
    return depthmap, imesh, jmesh, dmesh

############################################################################### 
#Cherche le maximum entre 2 points d'un signal
def cherche_idmax(list_x, list_y, start, end):
    if end - start > 1:
        dmax = 0
        imax = 0
        for i in range (int(end - start)):
            try :
                a = (list_x[end] - list_x[start])
                b = (list_y[end] - list_y[start])
                d = abs(a*(list_y[start] - list_y[start+i]) - b*(list_x[start] - list_x[start+i]))/ np.sqrt(a**2+b**2)
                if d > dmax:
                    dmax = d
                    imax = start+i
            except:
                print(len(list_x), start, end)
        return dmax, imax
    else:
        return 0, 0

###############################################################################

def filtr_angl(list_x, list_y, i_ret, start, end, t):
    if end-start <= 1:
        return [], []
    else:
        #print('start, end',start, end)
        dmax, imax = cherche_idmax(list_x, list_y, start, end)
        if dmax > t:
            fin, i_ret0 = filtr_angl(list_x, list_y, i_ret, imax, end, t)
            deb, i_ret0 = filtr_angl(list_x, list_y, i_ret, start, imax, t)
            #print('deb + [imax] +fin',[deb] + [imax] +[fin])
            i_ret.append(imax)
        return imax, i_ret