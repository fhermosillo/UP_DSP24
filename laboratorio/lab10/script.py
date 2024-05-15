# -*- coding: utf-8 -*-
"""
Created on Wed May 15 13:24:05 2024

@author: User123
"""

import numpy as np
import cv2
from kalman import KalmanFilter
from BallDetector import BallDetector
import matplotlib.pyplot as plt



# Get centroid
def get_centroid(bbox):
    return ((bbox[0]+bbox[2]-1)/2, (bbox[1]+bbox[3]-1)/2)



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
    
    # Kalman filter
    current_measurement = None
    current_prediction = np.array([[905], [311]])
    A = np.array()  # Dynamic Model (constant velocity)
    H = np.array()  # Measurement matrix (x,y)
    Q = np.array()  # Process noise covariance
    R = np.array()  # Measurement noise covariance
    x0=np.zeros((4,1))
    kalman = KalmanFilter(A=A,B=None,H=H,Q=Q,R=R,P=None,x0=None)
    kalman.update(np.array(current_prediction, dtype=np.float32))
    
    # Ball Detector (Template Matching)
    T = cv2.imread('ball.png')
    detector = BallDetector(T)
    
    while vid.isOpened():                             # read all video frames
        ret, frame = vid.read()
        if not ret:
            break
        
        bbox = detector.apply(frame)
        
        lpx, lpy = int(current_prediction[0]), int(current_prediction[1])
        frame = cv2.circle(frame, (lpx, lpy), 0, COLOR_RED, 20) # plot prediction dot
        x = kalman.predict()
        
        if bbox is not None: # detection -> draw bounding box, update kalman filter
            frame = cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), COLOR_GREEN, 2)
            # update kalman
            current_measurement = np.array(get_centroid(bbox), dtype=np.float32).reshape(2, 1)
            kalman.correct(current_measurement)
        
        
        current_prediction = (int(x[0]), int(x[1]))
        cv2.imshow("tracking", frame)
        cv2.waitKey(int(fps))
        #plt.imshow(frame, interpolation='nearest')
        #plt.show()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        out.write(frame)
    
    vid.release() # release video object
    out.release()