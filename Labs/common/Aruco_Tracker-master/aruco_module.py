"""
Framework   : OpenCV Aruco
Description : Calibration of camera and using that for finding pose of multiple markers
Status      : Working
References  :
    1) https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
    2) https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html
    3) https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
"""
import time

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
from scipy.spatial.transform import Rotation

# PATH = 'calib_images/tests/*.jpg'
# PATH  = r'C:\Users\admin\Documents\robotics_lab\Labs\common\Aruco_Tracker-master\calib_images/tests/*.jpg'
PATH = r'C:\Users\USER\Desktop\dev\robotics_lab\Labs\common\Aruco_Tracker-master\calib_images/tests/*.jpg'
class aruco_track():

    def __init__(self, channel=0, cbrow=6, cbcol=9, path=PATH, shape=aruco.DICT_4X4_250):

        self.cap = cv2.VideoCapture(channel, cv2.CAP_DSHOW)
        self.cbrow = cbrow
        self.cbcol = cbcol
        self.path = path
        self.shape = shape
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")
        self.calibrate()
    ####---------------------- CALIBRATION ---------------------------
    # termination criteria for the iterative algorithm
    def calibrate(self):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((self.cbrow * self.cbcol, 3), np.float32)
        objp[:,:2] = np.mgrid[0:self.cbcol, 0:self.cbrow].T.reshape(-1,2)

        objp = objp * 0.024
        # arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # iterating through all calibration images
        # in the folder
        images = glob.glob(self.path)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # find the chess board (calibration pattern) corners
            ret, corners = cv2.findChessboardCorners(gray, (self.cbcol,self.cbrow),None)

            # if calibration pattern is found, add object points,
            # image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                # Refine the corners of the detected corners
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (self.cbcol, self.cbrow), corners2,ret)


        self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    ###------------------ ARUCO TRACKER ---------------------------

    def track(self):

        ret, frame = self.cap.read()
        cv2.imshow('frame', frame)

        # operations on the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(self.shape)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.02, self.mtx, self.dist)

            for i in range(0, ids.size):
                # draw axis for the aruco markers
                aruco.drawAxis(frame, self.mtx, self.dist, rvec[i], tvec[i], 0.1)

            # draw a square around the markers
            aruco.drawDetectedMarkers(frame, corners)

            # code to show ids of the marker found
            strg = ''
            for i in range(0, ids.size):
                strg += str(ids[i][0])+', '

            cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            rvec = tvec = None

        # display the resulting frame
        cv2.imshow('frame',frame)

        return tvec, rvec, ids



if __name__ =='__main__':
    aru = aruco_track()
    counter = 0

    while (True):
        tvec, rvec, _ = aru.track()
        # time.sleep(1)
        if counter%50 and tvec is not None:
            # print('rvec: {}, tvec: {}'.format(rvec, tvec))
            t_curr, R_curr = tvec.squeeze(), rvec.squeeze()
            R_curr = Rotation.from_rotvec(R_curr).as_euler('xyz')
            print('rvec: {}, tvec: {}'.format(np.degrees(R_curr), t_curr))

        counter += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    aru.cap.release()
    cv2.destroyAllWindows()


