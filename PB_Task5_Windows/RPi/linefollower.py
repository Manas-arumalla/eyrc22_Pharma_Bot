'''
*****************************************************************************************
*
*        		===============================================
*           		Pharma Bot (PB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 4B-Part 1 of Pharma Bot (PB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_4b_1.py
# Functions:		control_logic, move_bot
# 					
####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section.   ##
## You have to implement this task with the available modules ##
##############################################################

import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
import sys
import datetime
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
from threading import Thread
from picamera.array import PiRGBArray
from picamera import PiCamera

########### ADD YOUR UTILITY FUNCTIONS HERE ##################
l1 = 12
l2= 13
en2 = 6
r1 = 20
r2 = 21
en1 = 26

Renc = 7
Lenc = 8
GPIO.setup(r1, GPIO.OUT)
GPIO.setup(r2, GPIO.OUT)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(l1, GPIO.OUT)
GPIO.setup(l2, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)

GPIO.setup(Renc, GPIO.IN)
GPIO.setup(Lenc, GPIO.IN)


pwm1 = GPIO.PWM(en1,100)
pwm2 = GPIO.PWM(en2,100)
pwm1.start(0)
pwm2.start(0)


mid = None
rs = ls = 0
cX = 0
angle = 0
lp = 0
END = False
##############################################################

def encoder_callback(channel):
    global angle
    angle += 18
    print(angle)
    
def measure_angle(enc):
    GPIO.add_event_detect(enc, GPIO.BOTH, callback=encoder_callback, bouncetime=25)
    

def forward(lspeed,rspeed):
    pwm1.start(rspeed)
    pwm2.start(lspeed)
    GPIO.output(r1,0)
    GPIO.output(r2,1)
    GPIO.output(l1,0)
    GPIO.output(l2,1)
def turn_inplace():
    pwm1.start(70)
    pwm2.start(70)
    GPIO.output(r1,1)
    GPIO.output(r2,0)
    GPIO.output(l1,0)
    GPIO.output(l2,1)
    
    
def stop():
    print('STOP')
    forward(0,0)
    
def left_turn():
    global angle, cX, mid, image, thresh
    print('LEFT')
    angle  = 0
    time.sleep(0.2)
    findAngle = Thread(target = measure_angle, args = [Renc])
    findAngle.start()
    
    while angle < 300:
        forward(53,0)
    stop()
    GPIO.remove_event_detect(Renc)
    findAngle.join()
    cX  = 0
    while cX < 100:
        detect_line(image, thresh)
        forward(53,0)
    stop()
#     cX = 0
        
    
    print('turned')
    
def right_turn():
    global angle, cX, mid, image, thresh
    print('RIGHT')
    angle  = 0
    time.sleep(0.2)
    findAngle = Thread(target = measure_angle, args = [Lenc])
    findAngle.start()
    
    while angle < 300:
        forward(0,53)
    stop()
    GPIO.remove_event_detect(Lenc)
    findAngle.join()
    cX = 400
    while cX > 380:
        detect_line(image, thresh)
        forward(0,53)
    stop()
#     cX = 400
    print('turned')
    
def reverse():
    global angle, cX, mid, image, thresh
    print('REVERSE')
    angle  = 0
    time.sleep(0.2)
    findAngle = Thread(target = measure_angle, args = [Lenc])
    findAngle.start()
    
    while angle < 360:
        turn_inplace()
    stop()
    GPIO.remove_event_detect(Lenc)
    findAngle.join()
    cX = 400
    while cX > 380:
        detect_line(image, thresh)
        turn_inplace()
    stop()
    cX = 400
    print('turned')

    
    
def wait():
    stop()
    print('WAIT')
    time.sleep(5)


def hsv_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([0, 100, 0])
    higher_hsv = np.array([42, 255, 255])
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

    frame = cv2.bitwise_and(img, img, mask=mask)
    kernel = np.ones((5,5),np.uint8)
    frame = cv2.erode(frame, kernel, iterations=1)

    return frame

def process_img(img):
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#         blur = cv2.blur(gray, (100,100))
# #         cv2.imshow('blur',blur)
        _ ,thresh=cv2.threshold(gray,90, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5),np.uint8)
        thresh = cv2.erode(thresh, kernel, iterations=1)
        cv2.imshow('thresh',thresh)
        mask = hsv_mask(img)
#         cv2.imshow('mask',mask)
        return thresh, mask
        
def detect_line(img, thresh):
    
    
    global mid, cX, angle, isLine
    try:
                

        y1 = y3 = 100
        x1 = np.where(thresh[y1] == 0)[0][0]
        y = np.where(thresh.T[0]==255)[0]
        if len(y) > 0:
            y2 = np.where(thresh.T[0]==255)[0][-1]
            x2 = np.where(thresh[y2] == 0)[0][0]
        else :
            y2 = 359
            x2 = 0
            
       
        
        x3 = np.where(thresh[y3] == 0)[0][-1]
        
        y = np.where(thresh.T[479]==255)[0]
        if len(y)> 0:
            y4 = np.where(thresh.T[479]==255)[0][-1]
            x4 = np.where(thresh[y4] == 0)[0][-1]
        else:
            y4 = 359
            x4 = 479
        cv2.line(img, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.line(img, (x3,y3), (x4,y4), (0,255,0), 2)
        
        cY = sum([y1,y2,y3,y4])//4
        cX = sum([x1,x2,x3,x4])//4
        
        cv2.circle(img , (cX,cY), 3, (255,0,0), 3)
        
#         #slope
#         Sx1 = sum([x1,x3])//2
#         Sx2 = sum([x2,x4])//2
#         Sy1 = sum([y1,y3])//2
#         Sy2 = sum([y2,y4])//2
#         
#         cv2.line(img , (Sx1,Sy1), (Sx2,Sy2), (0,255,255), 3)
#         cv2.line(img, (240,0), (240,359), (255,255,0), 2)
#         
#         angle = -np.arctan2(Sx2-Sx1, Sy2-Sy1 )*180/np.pi
        isLine = True
#         print(angle)
        
        
    except :
#         print('No LINE')
        isLine = False
#         stop()
#         if cX > mid[1]:
# #             cX = 300
#             rs = 0
#             ls = 50
#         else:
# #             cX = 180
#             rs = 50
#             ls = 0
#         pass
       
#

def detect_node(mask):
    mask = np.where(mask == 0 , mask , 1)
    if mask.sum() > 2500:
        return True
    return False
    
    
def follow_line():
    global cX, mid, rs, ls, isLine
    try:
        pid = PID(cX,mid,angle)
        
        speed=55
        if isLine:
            if abs(pid) < speed:
                if pid < 0:
                    rs = speed - pid
                    ls = speed
                    forward(rs,ls)
                elif pid > 0:
                    ls = speed + pid
                    rs = speed
                    forward(rs,ls)
                else:
                    rs = ls = speed
                    forward(speed,speed)
#                 print(rs,ls)
        else:
            if cX > mid[1]:
                print('Line out right')
                rs = 0
                ls = 45
            else:
                print('Line out left')
                rs = 45
                ls = 0
            forward(rs, ls)
            
    except: pass

            

    
  
def PID(cX,mid,angle):
#     print(cX, mid[1])
    global lp
#     print(cX)
    error = cX - mid[1]
#     print('errors:',error, angle)
    kp = 0.05
    kd = 1.5
    d = error-lp
    lp = error
    print('errors:',error, d)
    pid = kp*error +kd*d
    print('pid:',pid)
    
    return pid
##############################################################    

def control_logic():
    """
    Purpose:
    ---
    This function is suppose to process the frames from the PiCamera and
    check for the error using image processing and with respect to error
    it should correct itself using PID controller.

    >> Process the Frame from PiCamera 
    >> Check for the error in line following and node detection
    >> PID controller

    Input Arguments:
    ---
    You are free to define input arguments for this function.

    Hint: frame [numpy array] from PiCamera can be passed in this function and it can
        take the action using PID 

    Returns:
    ---
    You are free to define output parameters for this function.

    Example call:
    ---
    control_logic()
    """    

    ##################	ADD YOUR CODE HERE	##################
    global mask, mid, cX, thresh, image, END
    
    camera = PiCamera()
    camera.resolution = (480, 360)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(480, 360))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        mid = np.array(image.shape[:2])//2
        thresh, mask = process_img(image)
        
#         detect_line(image, thresh)
#         print('follow')
       
        
        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF 
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        # if the `q` key was pressed, break from the loop
        if key == ord("q") or END:
            stop()
            break

    ##########################################################

def move_bot(path):
    """
    Purpose:
    ---
    This function is suppose to move the bot

    Input Arguments:
    ---
    You are free to define input arguments for this function.

    Hint: Here you can have inputs left, right, straight, reverse and many more
        based on your control_logic

    Returns:
    ---
    You are free to define output parameters for this function.

    Example call:
    ---
    move_bot()
    """    

    ##################	ADD YOUR CODE HERE	##################
    global mask, mid, cX, image, thresh, angle, END
    
    
    directions = {'STRAIGHT':stop, 'RIGHT':right_turn, 'LEFT': left_turn, 'WAIT_5': wait, 'REVERSE':reverse }
    counter = 0
    directions[path[counter]]()
    time.sleep(1)
    print('ended node')
    
    
    print('node')
    counter+=1
    
    while True:
        if not detect_node(mask):
                detect_line(image, thresh)
                follow_line()

        else:
                stop()
                if counter >= len(path):
                    stop()
                    END = True
                    return
                time.sleep(1)
                while detect_node(mask):
                    forward(58,58)
                stop()
                print("h")
                angle  = 0
                time.sleep(0.2)
                findAngle = Thread(target = measure_angle, args = [Renc])
                findAngle.start()
                
                while angle < 170:
                    forward(58,58)
                stop()
                print("h")
    
                directions[path[counter]]()
                time.sleep(1)
                print('ended node')
                print('node')
                if path[counter] == 'WAIT_5':
                    counter+=1
                    directions[path[counter]]()
                    
                counter+=1


    ##########################################################



################# ADD UTILITY FUNCTIONS HERE #################





##############################################################
mask = thresh = image= None


if __name__ == "__main__":

    """
    The goal of the this task is to move the robot through a predefied 
    path which includes straight road traversals and taking turns at 
    nodes. 

    This script is to be run on Raspberry Pi and it will 
    do the following task.
 
    >> Stream the frames from PiCamera
    >> Process the frame, do the line following and node detection
    >> Move the bot using control logic

    The overall task should be executed here, plan accordingly. 
    """    

    ##################	ADD YOUR CODE HERE	##################

    image_process = Thread(target = control_logic)
    path = ['STRAIGHT','LEFT','WAIT','LEFT']
    line_follow = Thread(target = move_bot, args = [path])
    image_process.start()
    time.sleep(2)
    line_follow.start()

    ##########################################################





