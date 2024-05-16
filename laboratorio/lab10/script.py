# -*- coding: utf-8 -*-
"""
Created on Wed May 15 13:24:05 2024

@author: User123
"""

import numpy as np
import cv2
from kalman import KalmanFilter
from BallDetector import BallDetector


# Get centroid: FIXED CENTROID COMPUTATION
def get_centroid(bbox):
    return np.array([[bbox[0]+bbox[2]/2], [bbox[1]+bbox[3]/2]])



# Main Code
def ball_tracking():
    src='ball.mp4'
    dst='ball-tracker.mp4'
    vid = cv2.VideoCapture(src)
    if not vid.isOpened():
        print("File open error, please download video")
    
    # params for saving video
    fps = int(vid.get(cv2.CAP_PROP_FPS))              # fps
    height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))  # height
    width = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))    # width
    size = (width, height)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(dst, fourcc, fps, size)
    
    # Color in BGR
    COLOR_RED = (0, 0, 255)
    COLOR_GREEN = (0, 255, 0)
    COLOR_BLUE = (255, 0, 0)
    
    # Kalman filter
    z0 = np.array([[909], [283]], dtype=np.float32)
    zhat = z0
    dt = 0.025
    A = np.array([[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]], dtype=np.float32)  # Dynamic Model (constant velocity)
    H = np.array([[1,0,0,0],[0,1,0,0]], dtype=np.float32)  # Measurement matrix (x,y)
    Q = np.eye(4)*0.0005
    R = np.eye(2)*0.001
    kalman = KalmanFilter(A=A,B=None,H=H,Q=Q,R=R)
    kalman.correct(z0)
    
    # Ball Detector
    detector = BallDetector()
    
    cv2.namedWindow("Tracking") 
    while vid.isOpened():                             # read all video frames
        ret, frame = vid.read()
        if not ret:
            break
        
        # Perform ball detection
        bbox = detector.apply(frame)
        
        # Kalman prediction
        x = kalman.predict()
        
        if len(bbox): # If ball was detected, update kalman filter
            # Plot detected bounding box: BOUNDING BOX FOR DETECTION
            frame = cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), COLOR_GREEN, 2)
            
            # update kalman
            z = get_centroid(bbox)
            x = kalman.correct(z)
            zhat = np.dot(kalman.H,x)
            
            # Plot estimated centroid: CENTROID FOR ESTIMATION
            frame = cv2.circle(frame, (int(zhat[0]), int(zhat[1])), 0, COLOR_BLUE, 10) # plot prediction dot
            
        else:
            # Plot predicted centroid: CENTROID FOR PREDICTION
            zhat = np.dot(kalman.H,x)
            frame = cv2.circle(frame, (int(zhat[0]), int(zhat[1])), 0, COLOR_RED, 10)
        
        cv2.imshow("Tracking", frame)
        k = cv2.waitKey() & 0xFF
        if k == 27:
           break
       
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        out.write(frame)
    
    vid.release() # release video object
    out.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    ball_tracking()
