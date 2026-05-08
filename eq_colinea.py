import math
import numpy as np
from numpy.linalg import norm 

#XYZ to xy
def colinearite(F,S,R,M): 
    #print('M-S',M-S)
    RMS = R@(M-S)
    # print('RMS',RMS)
    l = F[2]/RMS[2]
    #print('l',l)
    m = F - l * RMS
    #print('m',m)
    return m

#center image
def toImageCoord(m,w,h):
    i = w/2-m[0]
    j = h/2+m[1]
    return i,j

def topixelCoord(m,w,h):
    i = m[0] - w/2
    j = h/2 - m[1]
    return i,j

#correction distorsions (cas caméras cadre)
def correct_disto_agisoft(x,y,f,cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h):
    x = x / f
    y = y / f
    r2 = pow(x,2)+pow(y,2)
    xp = x*(1+k1*r2+k2*pow(r2,2)+k3*pow(r2,3)+k4*pow(r2,4))+(p1*(r2+2*pow(x,2))+2*p2*x*y)*(1+p3*r2+p4*pow(r2,2))
    yp = y*(1+k1*r2+k2*pow(r2,2)+k3*pow(r2,3)+k4*pow(r2,4))+(p2*(r2+2*pow(y,2))+2*p1*x*y)*(1+p3*r2+p4*pow(r2,2))
    u = w/2+cx-xp*f-xp*b1-yp*b2
    v = h/2+cy+yp*f
    return u,v

#correction distorsions (cas caméras fisheye)
def correct_disto_agisoft_fisheye(x,y,f,cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h):
    x0 = x /f
    y0 = y /f
    # print('x0,y0',x0,y0)
    r0 = np.sqrt( pow(x0,2)+pow(y0,2) )
    x = x0 * (math.atan(r0))/r0
    y = y0 * (math.atan(r0))/r0
    r2 = pow(x,2)+pow(y,2)
    # print('x,y',x,y)
    xp = x*(1+k1*r2+k2*pow(r2,2)+k3*pow(r2,3)+k4*pow(r2,4))+(p1*(r2+2*pow(x,2))+2*p2*x*y)
    yp = y*(1+k1*r2+k2*pow(r2,2)+k3*pow(r2,3)+k4*pow(r2,4))+(p2*(r2+2*pow(y,2))+2*p1*x*y)
    u = w/2+cx-xp*f-xp*b1-yp*b2
    v = h/2+cy+yp*f
    # print('u,v',u,v)
    return u,v


def projection(M,S,R,cx,cy,f,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h):
    F = np.array([0,0,-f])
    m = colinearite(F,S,R,M)
    # print('m',m) # 3323, 1775
    return correct_disto_agisoft_fisheye(m[0],m[1],f,cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h)

def projection_sans_cor(M,S,R,cx,cy,f,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h):
    F = np.array([0,0,-f])
    m = colinearite(F,S,R,M)
    # print('m',m) # 3323, 1775
    return correct_disto_agisoft(m[0],m[1],f,cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h)

def unproject(ihoriz, jhoriz, dhoriz, F, S, R_temp, w=2720, h=2720):
    coord_img = [jhoriz, ihoriz]
    i_img, j_img = topixelCoord(coord_img,w,h)
    m = np.array([i_img, 0, j_img])
    M = dhoriz/norm(F) * R_temp@(m-F) + S
    return M[0], M[1], M[2]

# obs_valeur = [
#     {"X": 2540516.75, "Y":1181183.00, "Z":468.91, "x":1977, "y":731},
#     {"X": 2540511.50, "Y":1181189.25, "Z":468.88, "x":1742, "y":686},
#     {"X": 2540513.25, "Y":1181200.50, "Z": 468.88, "x":1470, "y":786},
#     {"X": 2540515.50, "Y":1181212.00, "Z": 468.91, "x": 1259, "y":877},
#     {"X": 2540513.50, "Y":1181222.63, "Z": 468.83, "x":1105, "y":952},
#     {"X": 2540518.50, "Y":1181234.25, "Z":466.03, "x":991, "y":1066},
    
#     {"X": 254092.50, "Y": 1181213.63, "Z":454.42, "x":2156, "y":1171},
#     {"X": 2540518.50,"Y": 1181180.13, "Z": 454.42, "x":678, "y":1228},
#     {"X": 2540487.75, "Y":1181201.25, "Z":448.72, "x":715, "y":1442},
#     {"X": 2540495.75, "Y":1181171.38, "Z":454.39, "x":2459, "y":945}]

# F = np.array([0,-1360,0])
# import sys
# sys.path.append( r'C:\Users\chahi\Desktop\INSA\G5\PFE\prog')
# import reframe as ref
# import eq_colinea as col
# import lecture as lec
# lien = r'C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\gopro\out\GS010194'

# [xmn95, ymn95, zmn95], index = lec.lecture_coord_cam("Frt_0140.png", r"C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\gopro\out\GS010194\pos.txt")
# xcam, ycam, zcam = ref.reframe(xmn95, ymn95, zmn95) 
# S = np.array([xcam, ycam, zmn95])
# R_camera = lec.lecture_matrix_cam("Frt_0140.png", lien+'\mat_rot.txt')
# col.unproject(1947, 731, 35, F, S, R_camera, w=2720, h=2720)


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





