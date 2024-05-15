# -*- coding: utf-8 -*-
"""
Created on Wed May 15 12:54:42 2024

@author: User123
"""

import cv2


class BallDetector(object):
    def __init__(self,thresHSV=100, thresVal=5, thresArea=300):
        self.thresHSV = thresHSV
        self.thresVal = thresVal
        self.thresArea = thresArea
        self.se = cv2.getStructuringElement(cv2.MORPH_RECT, (10,10))

    def apply(self, I):
        # Threshold HSV image
        Y=cv2.cvtColor(I,cv2.COLOR_RGB2HSV)
        _,M=cv2.threshold(cv2.absdiff(Y[:, :, 0], self.thresHSV), self.thresVal, 255, cv2.THRESH_BINARY_INV)
        # Filter image
        BW=cv2.erode(M, self.se)
        
        # Find contours
        contours,_ = cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #img = cv2.drawContours(frame, contours, -1, (0,255,0), 1)
        
        # Filter contours by area
        bboxes=[]
        maxArea = 0
        for i in range(len(contours)):
            area=cv2.contourArea(contours[i])
            if area > self.thresArea:
                if maxArea < area:
                    bboxes=cv2.boundingRect(contours[i])
        
        return bboxes
    

