import logging
import math 
import numpy as np
from osgeo import gdal


def computeCircularProfile(XY,Z,dtm,az_step=10,dtm_step=1,height_factor=1.0, az_0 = 0, FOV = 90):
    logging.debug("computeCircularProfile") 
    cp_height = Z
    pos = XY
    dat = {}
    pts = {}
    for i in range(0,2720,az_step):
        az = round(i * FOV/2720 + az_0-FOV/2, 4)
        dir = np.array([math.cos(math.radians(90-az)),math.sin(math.radians(90-az))])
        pt_cur = pos + dir*dtm_step
        h_cur = dtm.getNearestZ(pt_cur)
        dist = dtm_step
        elev = 0
        fMax = [-1,1,1]
        ptmax = pt_cur
        hmax = h_cur
        while h_cur != dtm.noDataValue:
            dZ = (h_cur - cp_height)*height_factor
            f = dZ/dist
            if f > fMax[0]:
                fMax[0] = dZ/dist
                fMax[1] = dZ
                fMax[2] = dist
                ptmax = pt_cur
                hmax = h_cur
                pts[round(az,4)] = [ptmax[0], ptmax[1], hmax]
                #elev = max(elev,math.degrees(max(math.atan2(dZ,dist),0)))
            pt_cur += dir*dtm_step
            dist += dtm_step
            h_cur = dtm.getNearestZ(pt_cur)#getBilinearZ
            elev = 90 - math.degrees(max(math.atan2(fMax[1],fMax[2]),0))  
            dat[az] = [elev, fMax[2]]
    return dat, pts



class DTM():
    def __init__(self,filename):
        dataset = gdal.Open(filename)
        if dataset is None:
            self.data = None
            return
        band = dataset.GetRasterBand(1)
        self.cols = dataset.RasterXSize
        self.rows = dataset.RasterYSize
        transform = dataset.GetGeoTransform()
        self.xOrigin = transform[0]
        self.yOrigin = transform[3]
        self.pixelWidth = transform[1]
        self.pixelHeight = -transform[5]
        self.noDataValue = -9999 #TODO : read in file ??
        self.data = band.ReadAsArray(0, 0, self.cols, self.rows)
        dataset = None
    # ###
    def getNearestZ(self,xy):
        col = int((xy[0] - self.xOrigin) / self.pixelWidth)
        row = int((self.yOrigin - xy[1]) / self.pixelHeight)
        if (0 <= col < self.cols) and (0 <= row < self.rows):
            return self.data[row][col]
        else:
            return self.noDataValue