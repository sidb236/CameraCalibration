
import opencv as cv2
import numpy as np
import glob
from criteria import CRITERIA
import Calibration as cal

"""This is the main file which calls the InternalCalibration 
    class and use it for calibration """

def main(CRITERIA, path):

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # in the checker board
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    images = glob.glob(path+'*.jpg')

    calibration_object= cal.InternalCalibration(objp, imgpoints, path)
    calibration_object.findcorners()
    ret, mtx, dist, rvecs, tvecs = calibration_object.calibratecamera()

    with open("CameraParamaters.txt") as f:
        f.write("mtx:"+ mtx )
        f.write("\n")
        f.write("dist:"+dist)
        f.write("\n")
        f.write("rvecs:" + rvecs)
        f.write("\n")
        f.write("tvecs:" + tvecs)


if__name__ =="__main__":
    main(CRITERIA, path='')
