# -*- coding: utf-8 -*-
"""
Created on Wed May 15 13:48:17 2024

@author: User123
"""

import cv2
from BallDetector import BallDetector


src='ball.mp4'
vid = cv2.VideoCapture(src)
if not vid.isOpened():
    print("File open error, please download video")

# Constant colors in BGR
COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)

# Ball Detector (Template Matching)
T = cv2.imread('ball.png')
detector = BallDetector()

#cv2.namedWindow("Tracking", cv2.WINDOW_NORMAL) 
cv2.namedWindow("Tracking") 

while vid.isOpened():
    # Read frame by frame
    ret, frame = vid.read()
    if not ret:
        break
    
    # Detect using template matching
    bbox = detector.apply(frame)
    
    
    # Draw detection in blue
    if len(bbox) > 0:
        frame = cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), COLOR_RED, 2)
    
    
    cv2.imshow("Tracking", frame)
    cv2.waitKey()

vid.release() # release video object
cv2.destroyAllWindows()

