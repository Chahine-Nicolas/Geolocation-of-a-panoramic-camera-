import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rota
import cv2
import imutils
import reframe as ref
import lecture as lec
import horizon as hor
import circular_profil as cir
from dtw import *



###############################################################################
###############################################################################
###############################################################################
###############################################################################

dtm_path = r"C:\Users\chahi\Desktop\INSA\G5\PFE\donnees\raster.tif"


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
    
    img_b2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)
    img_open2 = ndimage.binary_opening( img_b2, structure=np.ones((3,3))).astype(int) 
    iimg, jimg = hor.find_horizon(img_open2)

    jimgdz, iimgdz = zip(*sorted(zip(jimg, iimg)))
    jimg, iimg = list(jimgdz), list(iimgdz)
    
    #lecture raster
    az_0 = heading%360
    az_02 = (az_0+180)%360
    XY = (S[0], S[1])
    Z = S[2]
    dtm = cir.DTM(dtm_path)
    dat, pts = cir.computeCircularProfile(XY,Z, dtm, az_step=1, dtm_step=0.5, height_factor=1.0, az_0=az_0, FOV = FOV)
  
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
    
    MFrt, MBck = [], [] 
    for i in range(len(pts)):
        i = round(i * 90/2720+ az_0-FOV/2,4)
        MFrt.append([pts[i][0], pts[i][1], pts[i][2]])
    

    query2 = 1360-np.array(el_pcd)*27
    reference = np.array(iimgr) 
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
