#!/usr/bin/env python
import numpy as np
import cv2
#from matplotlib import pyplot as plt
import rospy
cap = cv2.VideoCapture(0)
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
 
 # Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
 
 # Create some random colors
color = np.random.randint(0,255,(100,3))
 
 # Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
p0_const = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
 # Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

while(True):
    # Capture frame-by-frame

    ret, frame = cap.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if len(p0)<3:
       p0=p0_const.copy()
     # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    
     # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]



    E, mask = cv2.findFundamentalMat( good_old, good_new,cv2.FM_LMEDS)
    pts1 = good_old[mask.ravel()==1]
    pts2 = good_new[mask.ravel()==1]

    
    print pts1
    
  






    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break


    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()




