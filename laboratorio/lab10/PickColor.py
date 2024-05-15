# -*- coding: utf-8 -*-
"""
Created on Wed May 15 15:23:30 2024

@author: User123
"""

import cv2


finished = False

Y = None
cref = None
# mouse callback function
def mouse_callback(event,x,y,flags,param):
   if event == cv2.EVENT_LBUTTONDOWN:
       if Y is not None:
           print("Y(",x, ",", y, ") = [", Y[y,x,0], ",", Y[y,x,1], ",", Y[y,x,2], "]")

cv2.namedWindow("HSV") 
cv2.setMouseCallback('HSV', mouse_callback) #Mouse callback

# Get image
src='ball.mp4'
vid = cv2.VideoCapture(src)
if not vid.isOpened():
    print("File open error, please download video")    
ret, frame = vid.read()
Y=cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)

# Get color
while True:
      cv2.imshow('HSV',Y)
      k = cv2.waitKey(4) & 0xFF
      if k == 27:
         break

cv2.destroyAllWindows()
   
   
