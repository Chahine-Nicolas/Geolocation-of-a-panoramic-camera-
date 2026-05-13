import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as Rota
from numpy.linalg import norm 

import convert_angle as cv
import lecture as lec

#cori = lecture_quat()
R = 6369000

def plot_angle(euler, num):
    Xroll = []
    Ypitch = []
    Zyaw = []
    for i in range (len(euler)):
        Xroll.append( euler[i][0])
        Ypitch.append( euler[i][1])
        Zyaw.append( euler[i][2])
    plt.figure()
    plt.xlabel("frames (SU)")
    plt.ylabel("angles (deg")
    plt.plot(Xroll, color='blue', label = 'X')  
    plt.plot(Ypitch, color='red', label = 'Y') 
    plt.plot(Zyaw, color='green', label = 'Z') 
    plt.title("Evolution des angles au cours d'une vidéo "+str(num))
    plt.legend()
    plt.savefig("Evolution des angles au cours d'une vidéo "+str(num))
    return 

def plot_trajectoire(link, num = 88):
    Coord = lec.lecture_coord(link)
    if Coord != []:
        print(True)
    E = []
    N = []
    for i in range (len(Coord)):
        e, n = cv.sph2EN('plate_carree', Coord[i][0], Coord[i][1], R)
        x,y,z = cv.sph2cart(Coord[i][0], Coord[i][1], R)
        E.append(e)
        N.append(n)      
    plt.figure()
    plt.scatter(E, N)
    plt.xlabel("Est (m)")
    plt.ylabel("Nord (m)")
    plt.axis('equal')
    plt.title("Trajectoire vidéo n°GS01"+f"{num:04d}")
    plt.savefig("Trajectoire vidéo n°GS01"+f"{num:04d}")
    return 

def plot_vecteur3D(ax, start, fin, colr, labl=""): 
    ax.quiver(start[0], start[1], start[2], fin[0], fin[1], fin[2], color=colr, label=labl)
    return

def plot_magn(MAGN, center, normmagn):
    MAGN = np.array(MAGN)
    
    # plt.figure()
    # plt.plot(MAGN[:,0],"r")
    # plt.plot(MAGN[:,1],"g")
    # plt.plot(MAGN[:,2],"b")
    # plt.title('MAGN')
    # plt.xlabel("Numéro de l'image")
    # plt.ylabel("Mesures du champ magnétique (µT)")
    # plt.show()
    
    MAGNcentered = MAGN-center
    normi = (MAGNcentered[:,0]**2+MAGNcentered[:,1]**2+MAGNcentered[:,2]**2)**0.5
    plt.figure()
    plt.plot(MAGNcentered[:,0]/normi,"r")
    plt.plot(MAGNcentered[:,1]/normi,"g")
    plt.plot(MAGNcentered[:,2]/normi,"b")
    plt.title('MAGNcentered')
    plt.xlabel("Numéro de l'image")
    plt.ylabel("Mesures du champ magnétique (µT)")
    plt.show()
    heading = np.arctan2(-MAGNcentered[:,1], MAGNcentered[:,0]) * (180 / np.pi)
    plt.figure()
    plt.title('Heading')
    plt.xlabel("Numéro de l'image")
    plt.ylabel("Azimut sud (deg)")
    plt.plot(heading)
    plt.show()
    return heading

def plot_coeff_mat_rot(list_mat_r, num):
    plt.figure()
    r11, r12, r13, r21, r22, r23, r31, r32,r33 = [], [], [], [], [], [], [], [], []
    x = []
    for j in range(len(list_mat_r)):
        r11.append(list_mat_r[j][0][0])
        r12.append(list_mat_r[j][0][1])
        r13.append(list_mat_r[j][0][2])
        
        r21.append(list_mat_r[j][1][0])
        r22.append(list_mat_r[j][1][1])
        r23.append(list_mat_r[j][1][2])
        
        r31.append(list_mat_r[j][2][0])
        r32.append(list_mat_r[j][2][1])
        r33.append(list_mat_r[j][2][2])
        
        x.append(j)
    plt.title('Evolution des coefficient des matrices rotations de la caméra '+str(num)+'list_mat_r')
    plt.xlabel('Indice caméra')
    plt.ylabel('valeur du coeff')
    plt.plot(x, r11, linewidth=10, linestyle=':', label='r11')
    plt.plot(x, r12, label='r12')
    plt.plot(x, r13, label='r13')
    
    plt.plot(x, r21, label='r21')
    plt.plot(x, r22, linewidth=4,linestyle=(0, (5, 2, 1, 2)), label='r22')
    plt.plot(x, r23, label='r23')
    
    plt.plot(x, r31, label='r31')
    plt.plot(x, r32, label='r32')
    plt.plot(x, r33, 'r--', label='r33')
    plt.legend()
    return

def triedre(ax, start, list_mat_R, i, gps_sync):
    vect_x = [list_mat_R[0][0], list_mat_R[1][0], list_mat_R[2][0]]
    vect_y = [list_mat_R[0][1], list_mat_R[1][1], list_mat_R[2][1]]
    vect_z = [list_mat_R[0][2], list_mat_R[1][2], list_mat_R[2][2]]
    if i != len(gps_sync)-1:
        plot_vecteur3D(ax, start, vect_x, 'red')
        plot_vecteur3D(ax, start, vect_y, 'blue')
        plot_vecteur3D(ax, start, vect_z, 'cyan')
    else:
        plot_vecteur3D(ax, start, vect_x, 'red', 'Xcam')
        plot_vecteur3D(ax, start, vect_y, 'blue', 'Ycam')
        plot_vecteur3D(ax, start, vect_z, 'cyan', 'Zcam')
    return

def triedre_Frt(ax, start, list_mat_R, i, gps_sync):
    vect_x = [list_mat_R[0][0], list_mat_R[1][0], list_mat_R[2][0]]
    vect_y = [list_mat_R[0][1], list_mat_R[1][1], list_mat_R[2][1]]
    vect_z = [list_mat_R[0][2], list_mat_R[1][2], list_mat_R[2][2]]
    if i != len(gps_sync)-1:
        plot_vecteur3D(ax, start, vect_x, 'red')
        plot_vecteur3D(ax, start, vect_y, 'darkorange')
        plot_vecteur3D(ax, start, vect_z, 'darkred')
    else:
        plot_vecteur3D(ax, start, vect_x, 'red', 'Xcam_Frt')
        plot_vecteur3D(ax, start, vect_y, 'darkorange', 'Ycam_Frt')
        plot_vecteur3D(ax, start, vect_z, 'darkred', 'Zcam_Frt')
    return

def triedre_Bck(ax, start, list_mat_R, i, gps_sync):
    vect_x = [list_mat_R[0][0], list_mat_R[1][0], list_mat_R[2][0]]
    vect_y = [list_mat_R[0][1], list_mat_R[1][1], list_mat_R[2][1]]
    vect_z = [list_mat_R[0][2], list_mat_R[1][2], list_mat_R[2][2]]
    if i != len(gps_sync)-1:
        plot_vecteur3D(ax, start, vect_x, 'blue')
        plot_vecteur3D(ax, start, vect_y, 'cyan')
        plot_vecteur3D(ax, start, vect_z, 'yellow')
    else:
        plot_vecteur3D(ax, start, vect_x, 'blue', 'Xcam_Bck')
        plot_vecteur3D(ax, start, vect_y, 'cyan', 'Ycam_Bck')
        plot_vecteur3D(ax, start, vect_z, 'yellow', 'Zcam_Bck')
    return