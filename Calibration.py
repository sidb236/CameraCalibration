import numpy as np
import cv2 as cv
import glob
from criteria import CRITERIA

# termination criteria
#criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) in the checker board
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('*.jpg')


class InternalCalibration:

    """Describe the Class and its functions"""

    def __init__(self, objpoints:list, imgpoints:list, path):
        self.objpoints = objpoints
        self.imgpoints = imgpoints
        self.path = path
        self.images = glob.glob(path+"*.jpg")
        self.mtx = None
        self.dist = None
        self.rvecs = None
        self.tvecs = None

    def findcorners(self):
        for fname in self.images:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, (7,6), None)

            # If found, add object points, image points (after refining them)

            if ret ==True:
                objpoints.append(self.objpoints)
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), CRITERIA)
                imgpoints.append(corners2)
                # Draw and display the corners
                cv.drawChessboardCorners(img, (7,6), corners2, ret)
                cv.imshow('img', img)
                cv.waitKey(500)
        cv.destroyAllWindows()

    def calibratecamera(self):
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        return ret, mtx, dist, rvecs, tvecs

    def unDistortimage(self):
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite('calibresult.png', dst)


    def undistortorrection(self):

        # undistort
        mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite('calibresult.png', dst)

    def calcmeanerror(self):
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "total error: {}".format(mean_error/len(objpoints)) )