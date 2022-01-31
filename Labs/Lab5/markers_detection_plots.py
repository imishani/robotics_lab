import cv2
import numpy
import numpy as np
import argparse
import time
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

# Global Parameters
car_ID = 10
axes_origin_ID = 0
s = time.time()

#--------------------------------------------------------------------------------------------------------------------
class arucoMarker:
    def __init__(self, x, y, distance, theta):
        self.x = x
        self.y = y
        self.distance = distance0
        self.theta = theta

    def updateTheta(self, rotate_theta):
        self.theta += rotate_theta

    def updatePosition(self, go_x, go_y):
        self.x += go_x
        self.y += go_y

#----------------------------------------------------------------------------------------------------------------

class cameraScreen:
    def __init__(self):
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        _, img = cap.read()
        self.arucosList, self.corners = detect(img,self.dictionary,self.parameters)
        cv2.destroyAllWindows()

        # calc edge center:  edge_center=(corner1+corner0)/2
        edge_centers = np.divide(np.add(self.corners[1], self.corners[0]), 2)
        centers = []

        for i in range(0, len(self.arucosList)):
            centers.append((self.arucosList[i]["cX"], self.arucosList[i]["cY"]))
        self.centersArr = np.array(centers)
        # calc direction vector: dir=edge_center-center
        self.dir = np.subtract(edge_centers, centers)
        # normalize direction vector: dir=dir/norm(dir)
        self.dir /= np.linalg.norm(self.dir)
        # calc angle relative to x axis: angle=arctan(y,x) --> (-pi,pi]
        self.orientation = np.arctan2(self.dir[0][0], self.dir[0][1])

    def updateGraph(self):

        for i in range(0, len(self.arucosList)):
            if self.arucosList[i]["id"] == axes_origin_ID:
                cX_0 = self.arucosList[i]["cX"]
                cY_0 = self.arucosList[i]["cY"]

        newCenters = []
        x, y = [], []
        for i in range(0, len(self.arucosList)):
            newCenters.append({"id": self.arucosList[i]["id"], "cX": self.arucosList[i]["cX"] - cX_0, "cY": self.arucosList[i]["cY"] - cY_0,"distance": ((self.arucosList[i]["cX"] - cX_0) ** 2 + (self.arucosList[i]["cY"] - cY_0) ** 2) ** 0.5})
            x.append(newCenters[i]["cX"])
            y.append(newCenters[i]["cY"])

        scatterGraph(x,y, f"Time is: {time.time()-s}")

#----------------------------------------------------------------------------------------------------------------

def scatterGraph(x, y, title):
    # plotting the points
    plt.scatter(x, y)
    plt.title(title)
    # naming the x axis
    plt.xlabel('x - axis')
    # naming the y axis
    plt.ylabel('y - axis')
    plt.show()

def detect(img, dictionary, parameters):
    marker_length = 0.025  # [m]
    camera_matrix = np.array([[1.42068235e+03, 0.00000000e+00, 9.49208512e+02],
                              [0.00000000e+00, 1.37416685e+03, 5.39622051e+02],
                              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    distortion_coeff = np.array([1.69926613e-01, -7.40003491e-01, -7.45655262e-03, -1.79442353e-03, 2.46650225e+00])

    corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

    res = []
    if len(corners) > 0:
        img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        ids = ids.flatten()

        for (corner, markerID) in zip(corners, ids):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distortion_coeff)
            distance = int(tvec[0][0][2] * 100)  # [cm]
            (topLeft, topRight, bottomRight, bottomLeft) = corner.reshape((4, 2))
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
            # cv2.rectangle(img, topLeft, bottomRight, (255,0,0), 2)
            # cv2.putText(img, str(markerID),
            #            (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
            #            0.5, (0, 255, 0), 2)
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            res.append({"id": markerID, "cX": cX, "cY": cY, "distance": distance})

            cv2.imshow("DEMO", img)
        key = cv2.waitKey(1)
        if key == ord('e'):
            cap.release()
            cv2.destroyAllWindows()

    return res,corners

#--------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":

    myScreen = cameraScreen()
    print(myScreen.arucosList)

    #Test
    next = 1
    while (next):
        for i in range(0, len(myScreen.arucosList)):
            if myScreen.arucosList[i]["id"] == car_ID:
                carTheta = np.arctan2(myScreen.dir[0][i][1], myScreen.dir[0][i][0])
                car = arucoMarker(myScreen.arucosList[i]["cX"], myScreen.arucosList[i]["cY"], myScreen.arucosList[i]["distance"], carTheta)
                car.updatePosition(float(input("go_x: ")), float(input("go_y: ")))
                myScreen.arucosList[i]["cX"] = car.x
                myScreen.arucosList[i]["cY"] = car.y

        myScreen.updateGraph()
        next = int(input("0 to STOP or 1 to CONTINUE? "))


