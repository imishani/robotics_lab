#!/usr/bin/env python
import numpy as np
import cv2

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cbrow = 6
cbcol = 9

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cbrow * cbcol, 3), np.float32)
objp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
i = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    i +=1
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (cbcol, cbrow),None)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.imwrite("test_osher/capture" + str(i) + ".jpg", frame)

        img = cv2.drawChessboardCorners(frame, (cbcol, cbrow), corners2,ret)
        cv2.imshow('frame',img)
        cv2.waitKey(250)
    else :
        cv2.imshow('frame',frame)
        cv2.waitKey(250)

    if len(imgpoints) >= 150:
        print ("calibrating camera now")
        cv2.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        np.savez("test_osher/MBP", ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()