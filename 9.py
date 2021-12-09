import cv2
import numpy as np
from walle.core import RotationMatrix

def rotz2angle(rotz):
    """Extracts z-rotation angle from rotation matrix.

    Args:
        rotz: (ndarray) The (3, 3) rotation about z.
    """
    return np.arctan2(rotz[1, 0], rotz[0, 0])


def clip_uv(uv, rows, cols):
    """Ensures pixel coordinates are within image bounds.
    """
    uv[:, 0] = np.clip(uv[:, 0], 0, rows - 1)
    uv[:, 1] = np.clip(uv[:, 1], 0, cols - 1)
    return uv

def rotate_uv(uv, angle, rows, cols, cxcy=None):
    """Finds the value of a pixel in an image after a rotation.

    Args:
        uv: (ndarray) The [u, v] image coordinates.
        angle: (float) The rotation angle in degrees.
    """
    txty = [cxcy[0], cxcy[1]] if cxcy is not None else [(rows // 2), (cols // 2)]
    txty = np.asarray(txty)
    uv = np.array(uv)
    aff_1 = np.eye(3)
    aff_3 = np.eye(3)
    aff_1[:2, 2] = -txty
    aff_2 = RotationMatrix.rotz(np.radians(angle))
    aff_3[:2, 2] = txty
    affine = aff_3 @ aff_2 @ aff_1
    affine = affine[:2, :]
    uv_rot = (affine @ np.hstack((uv, np.ones((len(uv), 1)))).T).T
    uv_rot = np.round(uv_rot).astype("int")
    uv_rot = clip_uv(uv_rot, rows, cols)
    return uv_rot

if __name__ == '__main__':
    way = 2
    if way == 1:
    #img = cv2.imread("1.png")
    # pts1 = np.float32([[50.4306, -175.9408], [199.0511, -175.1985], [195.3472, 34.5088]]) #[106.8482, 32.2865]
    # pts2 = np.float32([[186,105], [186,516], [764,500] ])#[759,263]     [289,424] [458,426]
    # pts3 = np.float32([[289,424]])
        imgpoints = [[11,71], [121.6,72], [231.3,286]]
        objpoints = [[31.31, -175.66],[34.31,-116.086], [155.4, -61.19]]
        pts1 = np.array(imgpoints, dtype='float32')
        pts2 = np.array(objpoints, dtype='float32')
        M = cv2.getAffineTransform(pts1, pts2)      # 第一种方式：三点标定法


    if way == 2:
        imgpoints = [[390,333], [547,339], [546,197], [41,321], [211,336], [39,12], [584,447]]
        objpoints = [[179.305, 17.204], [185.1776, 94.6872], [115.0382, 96.4722], [162.6492,-144.8213], [173.095,-68.6748], [17.7836, -139.3039], [240.0178, 108.7411]]
        imgpoints = np.array(imgpoints,dtype='float32')
        objpoints = np.array(objpoints,dtype='float32')
        M, _ = cv2.estimateAffine2D(imgpoints, objpoints,True)      # 第二种方式：九点标定法
    else: 
        print("error: no method specified!")
        raise RuntimeError("you need to enter 1 or 2.")


    pts3 = np.float32([[68,192]])
    print("M = ",M)
    angle = rotz2angle(M)
    angle = 180 * angle / 3.1415926
    print("angle = ",int(angle))
    R = M[:,:2]
    T = M[:,2]
    value_after_rot = (M @ np.hstack((pts3, np.ones((len(pts3), 1)))).T).T
    
    print(value_after_rot)