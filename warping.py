import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rota
import cv2
import imutils
import sys
sys.path.append( r'C:\Users\chahi\Desktop\INSA\G5\PFE\prog')
import reframe as ref
import lecture as lec
import horizon as hor
import circular_profil as cir


from dtw import *



###############################################################################
##############################################################################
# [xmn95, ymn95, zmn95], index = lec.lecture_coord_cam("Frt_0080.png", r"C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\gopro\out\GS010194\pos.txt")
# xcam, ycam, zcam = ref.reframe(xmn95, ymn95, zmn95) 
# mesh = o3d.io.read_triangle_mesh(r'C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\Lidar\swisssurface3d_2019_2540-1181_2056_5728/tile.ply')
# lien = r'C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\gopro\out\GS010194'
# # R_temp = np.array([[ 0.39743524,  0.91724149, -0.0084841 ],
# #               [-0.91517746,  0.39833159,  0.0693987 ],
# #               [ 0.06704808 , 0,          0.99755293]])
 
# # R_tempb = lec.lecture_matrix_cam("Frt_0120.png", lien+'\mat_rot.txt')
# R_camera = lec.lecture_matrix_cam("Frt_0080.png", lien+'\mat_rot.txt')
# R_agisoft = R_camera @ np.array([[1,0,0],
#                               [0,0,1],
#                               [0,-1,0]])
# r = Rota.from_matrix(R_agisoft).as_euler('xyz', degrees=True)
# FOV = 90

# S = np.array([xcam, ycam, zmn95])
# # camera front GorproMax perso
# cx,cy,f = 0.240779, -0.0230233, 1366.95
# F = np.array([0,-1360,0])
# k1,k2,k3,k4 = -0.170293, 0.00611307, 0.00174043, -0.000329237  
# b1,b2 = 0,0
# p1,p2,p3,p4 =  6.44318e-05, 0.000241483, 0, 0
# w,h = 2720,2720
# intrins = cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h

# depth_img = '\Frt_0080_mask.png'
[xmn95, ymn95, zmn95], index = lec.lecture_coord_cam("Frt_0270.png", r"C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\gopro\out\GS010190\pos.txt")
xcam, ycam, zcam = ref.reframe(xmn95, ymn95, zmn95) 
mesh = o3d.io.read_triangle_mesh(r'C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\Lidar\swisssurface3d_2019_2540-1181_2056_5728/tile.ply')
lien = r'C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\gopro\out\GS010190'
# R_temp = np.array([[ 0.39743524,  0.91724149, -0.0084841 ],
#               [-0.91517746,  0.39833159,  0.0693987 ],
#               [ 0.06704808 , 0,          0.99755293]])

# R_tempb = lec.lecture_matrix_cam("Frt_0120.png", lien+'\mat_rot.txt')
R_camera = lec.lecture_matrix_cam("Frt_0270.png", lien+'\mat_rot.txt')
R_agisoft = R_camera @ np.array([[1,0,0],
                              [0,0,1],
                              [0,-1,0]])
r = Rota.from_matrix(R_agisoft).as_euler('xyz', degrees=True)
FOV = 90
S = np.array([xcam, ycam, zmn95])
M = []
# camera front GorproMax perso
cx,cy,f = 0.240779, -0.0230233, 1366.95
F = np.array([0,-1366,0])
k1,k2,k3,k4 = -0.170293, 0.00611307, 0.00174043, -0.000329237  
b1,b2 = 0,0
p1,p2,p3,p4 =  6.44318e-05, 0.000241483, 0, 0
w,h = 2720,2720
intrins = cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h

depth_img = '\Frt_0270_mask.png'
###############################################################################
###############################################################################
###############################################################################
###############################################################################

def warping(lien, depth_img, FOV, S, R_temp, intrins, heading):
    #ji image
    
    up = np.array([R_temp[0][2], R_temp[1][2], R_temp[2][2]])
    alpha_up = np.arctan(up[0]/ up[2])*180/np.pi

    img = cv2.imread(lien+"\\"+depth_img)
    plt.figure()
    plt.imshow(img)
    plt.title("Ligne d'horizon de l'image")
    plt.xlabel('x')
    plt.ylabel('y')
    plt.savefig("av.png")
    
    

    

    #Image inclinée pour le warping
    Rotated_image = imutils.rotate(img, angle=alpha_up)
    img_b = cv2.cvtColor(Rotated_image, cv2.COLOR_BGR2GRAY).astype(np.uint8)
    img_open = ndimage.binary_opening( img_b, structure=np.ones((3,3))).astype(int) 
    iimgr, jimgr = hor.find_horizon(img_open)

    jimgrdz, iimgrdz = zip(*sorted(zip(jimgr, iimgr)))
    jimgr, iimgr = list(jimgrdz), list(iimgrdz)
    
    plt.figure('find_horizon')
    plt.scatter(jimgr, iimgr)
    plt.title("Ligne d'horizon de l'image2")
    plt.xlabel('x')
    plt.ylabel('y')
    plt.imshow(img_b, cmap='gray')
    plt.savefig("ap2.png")
    
    img_c = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)
    img_open = ndimage.binary_opening( img_c, structure=np.ones((3,3))).astype(int) 
    iimgr, jimgr = hor.find_horizon(img_open)

    jimgrdz, iimgrdz = zip(*sorted(zip(jimgr, iimgr)))
    jimgr, iimgr = list(jimgrdz), list(iimgrdz)
    
    plt.figure('find_horizon2 ')
    plt.scatter(jimgr, iimgr)
    plt.title("Ligne d'horizon de l'image")
    plt.xlabel('x')
    plt.ylabel('y')
    plt.imshow(img_c, cmap='gray')
    plt.savefig("ap1.png")
    
     
    
    
    
    #image d'origine pour 
    img_b2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)
    img_open2 = ndimage.binary_opening( img_b2, structure=np.ones((3,3))).astype(int) 
    iimg, jimg = hor.find_horizon(img_open2)

    jimgdz, iimgdz = zip(*sorted(zip(jimg, iimg)))
    jimg, iimg = list(jimgdz), list(iimgdz)
    
    # plt.figure('find_horizon')
    # plt.scatter(jimg, iimg)
    # plt.title("Ligne d'horizon de l'image")
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.imshow(img_b, cmap='gray')
    
    
    
 
    
    #lecture raster
    az_0 = heading%360
    az_02 = (az_0+180)%360
    XY = (S[0], S[1])
    Z = S[2]
    dtm = cir.DTM(r"C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\raster.tif")
    dat, pts = cir.computeCircularProfile(XY,Z, dtm, az_step=1, dtm_step=0.5, height_factor=1.0, az_0=az_0, FOV = FOV)
    # dat2, pts2 = cir.computeCircularProfile(XY,Z, dtm, az_step=1, dtm_step=0.5, height_factor=1.0, az_0=az_02, FOV = FOV)
    
    #coord polaire Frt
    az_pcd, el_pcd, d_pcd = [], [], []
    for i in range(int(len(dat))):
        i = round((i * 90/2720+ az_0-FOV/2),4)
        az_pcd.append(i%360)
        try :
            el_pcd.append(round((90-dat[i][0]),4))
            d_pcd.append(round(dat[i][1],4))
        except :
            print(i)
    plt.figure()
    plt.title('Horizon Frt  (circulaire)')
    plt.scatter(az_pcd, el_pcd, c=d_pcd)
    plt.axis('equal')
    plt.xlabel('azimuth')
    plt.ylabel('élévation')
    

    
    # R = np.array([
    #             [np.cos(alpha_up),-np.sin(alpha_up)],
    #             [np.sin(alpha_up),np.cos(alpha_up)]])
    
    # az_rot, el_rot = [], []
    # for i in range(int(len(dat))):
    #     xycur = np.array([az_pcd[i], el_pcd[i]])
    #     [azt, elt] = R@xycur
    #     az_rot.append(azt)
    #     el_rot.append(elt)

    # plt.figure()
    # plt.title('Horizon Frt  (circulaire)')
    # plt.scatter(az_pcd, el_pcd, color='red')
    # plt.scatter(az_rot, el_rot, color='blue')
    # plt.axis('equal')
    # plt.xlabel('azimuth')
    # plt.ylabel('élévation')
    
    # az_pcd = az_rot
    # el_pcd = el_rot
    
    
    # f1 = h / (2*math.tan(math.radians(FOV)/2))
    # imesh_2_fish, jmesh_2_fish = [], []
    # for i in range (len(imesh)):
    #     jcoor,icoor = col.correct_disto_agisoft_fisheye((np.max(az_pcd)/2-az_pcd[i])*30 ,(-el_pcd[i]+np.min(el_pcd)/2)*30,f1,cx,cy,k1,k2,k3,k4,p1,p2,p3,p4,b1,b2,w,h)
    #     if int(jcoor) in jmesh_2_fish :
    #         jmesh_2_fish[jmesh_2_fish.index(int(jcoor))] = jcoor
    #         imesh_2_fish[jmesh_2_fish.index(int(jcoor))] = icoor
    #     else:
    #         jmesh_2_fish.append(float(jcoor))
    #         imesh_2_fish.append(float(icoor))

    # plt.figure()
    # plt.title('Horizon Frt  (circulaire)')
    # plt.scatter(jimgr, iimgr)
    # plt.scatter(jmesh_2_fish,imesh_2_fish)
    # plt.xlabel('azimuth')
    # plt.ylabel('élévation')
    # plt.imshow(img)
    
    #coord polaire Bck
    # az_pcd2, el_pcd2, d_pcd2 = [], [], []
    # for i in range(int(len(dat2))):
    #     i = round((i * 90/2720+ az_02-FOV/2)%360,4)
    #     az_pcd2.append(i)
    #     el_pcd2.append(round((90-dat2[i][0]),3))
    #     d_pcd2.append(round(dat2[i][1],3))
    # plt.figure()
    # plt.title('Horizon Bck  (circulaire)')
    # plt.scatter(az_pcd2, el_pcd2, c=d_pcd2)
    # plt.axis('equal')
    # plt.xlabel('azimuth')
    # plt.ylabel('élévation')
    

    MFrt, MBck = [], []
    ##file5 = open(r"C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\pt_extrait.txt", "w") 
    for i in range(len(pts)):
        i = round(i * 90/2720+ az_0-FOV/2,4)
        ##file5.write(str(pts[i][0])+" "+str(pts[i][1])+" "+str(pts[i][2])+"\n")
        MFrt.append([pts[i][0], pts[i][1], pts[i][2]])
        #j = round(i + 180,4)
        ##file5.write(str(pts2[j][0])+" "+str(pts2[j][1])+" "+str(pts2[j][2])+"\n")
        #MBck.append([pts2[j][0], pts2[j][1], pts2[j][2]])
    ##file5.close() 
    
    

    query2 = 1360-np.array(el_pcd)*27
    reference = np.array(iimgr) 
    # reference = np.array(imesh_2_fish) 
    alignment = dtw(query2, reference, open_begin=True, open_end=True,  keep_internals=True, step_pattern='asymmetric')
    alignment.plot(type="twoway")
    
    
    alignment4 = dtw(query2, reference, open_begin=True, open_end=True,  keep_internals=True, step_pattern=rabinerJuangStepPattern(6, "c") )
    alignment4.plot(type="twoway")
    
    
    index2 = alignment.index2

    # appairer
    newi, newj, newM = [], [], []
    newi2, newj2 = [], []
    for k in range(len(index2)):
        newi.append( iimgr[index2[k]])
        
        newM.append( MFrt[k])
        newj.append( jimgr[index2[k]])
        try :
            newi2.append( iimg[index2[k]])
            newj2.append( jimg[index2[k]])
        except :
            newi2.append( iimgr[index2[k]])
            newj2.append( jimgr[index2[k]])
            
            
    #tri
    newjdz, newidz, newMdz = zip(*sorted(zip(newj, newi, newM)))
    newj, newi, newj = list(newjdz), list(newidz), list(newMdz)
    
    newj2dz, newi2dz= zip(*sorted(zip(newj2, newi2)))
    newj2, newi2= list(newj2dz), list(newi2dz)
    
    # plt.figure('horiz_final (pixel)')
    # plt.scatter(newj, newi)
    # plt.imshow(img)
    # plt.xlabel('J')
    # plt.ylabel('I')
    # plt.title("Ligne d'horizon appairée")

    
    # save pt appuis
    #file4 = open(lien +"\s#file_3_jiXYZimag.txt", "w") 
    obs = []
    for k in range (len(newM)):
          #file4.write(str(newM[k][0])+" "+str(newM[k][1])+" "+str(newM[k][2])+" "+str(newj[k])+" "+str(newi[k])+"\n")
          obs.append({'X':str(newM[k][0]),'Y': str(newM[k][1]),'Z': str(newM[k][2]),'x': str(newj[k]),'y': str(newi[k]) })
    #file4.close()

    t=20
    i_ret = []
    temp, i_ret = hor.filtr_angl(newj2, newi2, i_ret, 0, len(jimgr)-1, t)
    
    
    plt.figure() 
    plt.scatter(newj2, newi2)
    plt.xlabel('J')
    plt.ylabel('I')
    plt.imshow(img_b2, cmap='gray')
    
    plt.figure() 
    plt.scatter(newj2, newi2)
    plt.scatter(newj2[0], newi2[0], color='darkred', s=100)
    plt.scatter(newj2[-1], newi2[-1], color='darkred', s=100)
    for i in range(len(i_ret)):
        plt.scatter(newj2[i_ret[i]], newi2[i_ret[i]], color='red')
    plt.xlabel('J')
    plt.ylabel('I')
    plt.title("Filtrage des points d'horizon, Azimut sud="+str(heading))
    plt.imshow(img_b2, cmap='gray')

    obs2 = []
    #file7 = open(lien +"\mrkrfltr.txt", "w") 
    for i in range (len(i_ret)):
          #file7.write(str(newM[i_ret[i]][0])+" "+str(newM[i_ret[i]][1])+" "+str(newM[i_ret[i]][2])+"\n")
          obs2.append({'X':str(newM[i_ret[i]][0]),'Y': str(newM[i_ret[i]][1]),'Z': str(newM[i_ret[i]][2]),'x': str(newj2[i_ret[i]]),'y': str(newi2[i_ret[i]]) })
    #file7.close()


    return obs2
    
    
warping(lien, depth_img, FOV, S, R_agisoft, intrins, heading =  50)    
