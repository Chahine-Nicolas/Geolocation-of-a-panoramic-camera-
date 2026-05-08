import numpy as np
import numba as nb


def splitImageGopro(img):
    center = img[:, 688:3408]
    left = np.rot90(img[:, 0:688], k=-1)
    right = np.rot90(img[:, 3408:], k=-1)
    return center, left, right


def mergeWrapGopro(img1, img2):
    c1, l1, r1 = splitImageGopro(img1)
    c2, l2, r2 = splitImageGopro(img2)
    o1 = np.zeros((2720, 2720, 3), dtype=np.uint8)
    o1[688:-688, :, :] = c1
    o1[:688, 688:-688, :] = r2
    o1[-688:, 688:-688, :] = l2
    o2 = np.zeros((2720, 2720, 3), dtype=np.uint8)
    o2[688:-688, :, :] = c2
    o2[:688, 688:-688, :] = r1
    o2[-688:, 688:-688, :] = l1
    return o1, o2


@nb.njit
def xyFromPhiTheta(phi, theta):
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)

    a = np.max(np.array([np.abs(x), np.abs(y), np.abs(z)]))

    # Vector Parallel to the unit vector that lies on one of the cube faces
    xa = x / a
    ya = y / a
    za = z / a

    xPixel = 0
    yPixel = 0
    if (xa == 1):
        # Front OK
        u = 4/np.pi*np.arctan(za)
        v = 4/np.pi*np.arctan(ya)
        xPixel = int((1. - (u + 1.) / 2.) * 1344) + 688
        yPixel = int((1. - (v + 1.) / 2.) * 1344) + 688
    elif (za == 1):
        # Up OK
        u = 4/np.pi*np.arctan(xa)
        v = 4/np.pi*np.arctan(ya)
        xPixel = int(((u + 1.) / 2.) * 1344) - 656
        yPixel = min(2031, int((1. - (v + 1.) / 2.) * 1344) + 688)
    elif (ya == 1):
        # Left OK
        u = 4/np.pi*np.arctan(za)
        v = 4/np.pi*np.arctan(xa)
        xPixel = int((1. - (u + 1.) / 2.) * 1344) + 688
        yPixel = int((((v + 1.) / 2.)) * 1344) - 656
    elif (ya == -1):
        # Right OK
        u = 4/np.pi*np.arctan(za)
        v = 4/np.pi*np.arctan(xa)
        xPixel = min(2031, int((1. - (u + 1.) / 2.) * 1344) + 688)
        yPixel = int((1. - (v + 1.) / 2.) * 1344) + 688 + 1344
    elif (za == -1):
        # Down OK
        u = 4/np.pi*np.arctan(xa)
        v = 4/np.pi*np.arctan(ya)
        xPixel = int((1. - (u + 1.) / 2.) * 1344) + 688 + 1344
        yPixel = min(2031, int((1. - (v + 1.) / 2.) * 1344) + 688)
    return xPixel, yPixel


#@nb.njit
def createFishEyeIndex(outputSize):
    out = np.zeros((outputSize, outputSize, 2), dtype=np.uint16)
    w, h, _ = out.shape
    # phi, thet; # Polar coordinates
    for j in nb.prange(outputSize):
        v = 1-2*j / outputSize
        for i in nb.prange(outputSize):
            u = 1-2*i / outputSize
            theta = np.arccos(u)
            su = np.sin(theta)
            if (su == 0):
                continue
            rr = v/su
            if (-1 < rr < 1):
                phi = np.arcsin(rr)
                xPixel, yPixel = xyFromPhiTheta(phi, theta)
                if (0 <= xPixel < w) and (0 <= yPixel < h):
                    out[i, j] = [xPixel, yPixel]
    return out


@nb.njit
def convertToFishEyeFromIndex(src, index):
    out = np.ones((index.shape[0], index.shape[1], 3), dtype=np.uint8)*255
    w, h, _ = src.shape
    for j in range(index.shape[0]):
        for i in range(index.shape[1]):
            xPixel, yPixel = index[i, j]
            out[i, j] = src[xPixel, yPixel]

    return out


def unstitchGopro(f0, f5, outputSize, index=None):
    o1, o2 = mergeWrapGopro(f0, f5)
    if index is None:
        out1 = convertToFishEye(o1, outputSize)
        out2 = convertToFishEye(o2, outputSize)
    else:
        out1 = convertToFishEyeFromIndex(o1, index)
        out2 = convertToFishEyeFromIndex(o2, index)
    return out1, out2