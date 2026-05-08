import numpy as np


def calib_magn(MAGN):
    MAGN = np.array(MAGN)
    mMin = np.amin(MAGN, axis=0)
    mMax = np.amax(MAGN, axis=0)
    center = (mMin+mMax)/2
    MAGNcentered = MAGN-center
    norm = (MAGNcentered[:,0]**2+MAGNcentered[:,1]**2+MAGNcentered[:,2]**2)**0.5
    return center, norm



def heading(MAGN, center, normmagn):
    MAGN = np.array(MAGN)
    MAGNcentered = MAGN-center
    heading = np.arctan2(-MAGNcentered[:,1], MAGNcentered[:,0]) * (180 / np.pi) #- 3.34
    return heading