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

imgpoints = [[11,71], [9,179], [7.6,289],[121.6,72], [121,179], [120.3,287], [230.8,74], [231,180], [231.3,286]]
objpoints = [[31.31, -175.66], [90.87,-178.82], [150.21,-181.61], [34.31,-116.086], [93.66,-118.59], [152.495,-121.854], [59.44,-2.57], [95.96, -58.62], [155.4, -61.19]]
imgpoints = np.array(imgpoints,dtype='float32')
objpoints = np.array(objpoints,dtype='float32')
# imgpoints = np.array(imgpoints,dtype='float32').T         # halcon method
# objpoints = np.array(objpoints,dtype='float32').T
# column_robot = objpoints(0)
# row_robot = objpoints(1)

matri, _ = cv2.estimateAffine2D(imgpoints, objpoints,True)
print(matri)

# if __name__ == '__main__':
    
    