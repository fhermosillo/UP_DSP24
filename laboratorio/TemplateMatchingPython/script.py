# -*- coding: utf-8 -*-
"""
Created on Wed May  8 14:54:05 2024

@author: User123

https://www.addictinggames.com/clicker/kick-ya-chop
"""
import keyboard
import mss
import cv2
import numpy as np
from time import time, sleep
import pyautogui


# Region of Interest
dims = {
        'left': 460,
        'top': 290,
        'width': 920-460,
        'height': 900-290
    }

# Template loading
T_right = cv2.imread('tree.png')
T_left=cv2.flip(T_right,1)
T=T_right
w = T.shape[1]
h = T.shape[0]

# Screen capture
cap = mss.mss()
I = np.array(cap.grab(dims))
I = I[:,:,:3]

# Correlation
R = cv2.matchTemplate(I, T, cv2.TM_CCOEFF_NORMED)

# Get max value
_, r_max, _, max_loc = cv2.minMaxLoc(R)

# Thresholding
if r_max > 0.85:
    I=np.ascontiguousarray(I, dtype=np.uint8)
    cv2.rectangle(I, max_loc, (max_loc[0] + w, max_loc[1] + h), (0,255,255), 2)

# Show results
cv2.imshow("frame",I)
cv2.waitKey()
cv2.destroyAllWindows()



#keyboard.wait('s')
pyautogui.click(x=20, y=30)
# pyautogui.moveTo(100, 100, duration=0.25)
# wh = pyautogui.size()
#pyautogui.press('enter')
#pyautogui.write('\n')
# while True:
#     CurserPos = pyautogui.position()
#     print(CurserPos)
