#!/usr/bin/env python
import numpy as np
import cv2
#import glob
from matplotlib import pyplot as plt
import rospy

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
cap = cv2.VideoCapture(0)
while(True):
    retu, img = cap.read()
    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
    cv2.imshow('img',img)
    cv2.waitKey(500)
    

cv2.destroyAllWindows()

