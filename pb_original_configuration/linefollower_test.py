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

# Team ID:			PB_1996
# Author List:		VYBHAV RAGAVENDRA DEVARAKONDA, MANAS REDDY.A, VIGNESH.M, GNANESH.K
# Filename:			task_6.py
# Theme:			PHARMABOT
# Functions:		encoder_callback(channel), measure_angle(enc), forward(lspeed,rspeed), turn_inplace(), 
#                   stop(), left_turn(), right_turn(), reverse(), wait(), hsv_mask(img), process_img(img), 
#                   detect_line(img, thresh), detect_node(mask), follow_line(), PID(cX,mid,angle), 
#                   control_logic(), move_bot(path)
# Global variables:	l1 = 12
#                   l2= 13
#                   en2 = 6
#                   r1 = 20
#                   r2 = 21
#                   en1 = 26
#                   Renc = 7
#                   Lenc = 8
#                   mid = None
#                   rs = ls = 0
#                   cX = 0
#                   angle = 0
#                   lp = 0
#                   END = False
#                   pwm1 = GPIO.PWM(en1,100)
#                   pwm2 = GPIO.PWM(en2,100)
#                   mask = thresh = image= None
#                   mask, mid, cX, image, thresh, angle, END
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

# Function Name:encoder_callback
# Input:		channel (an integer representing the channel number)
# Output:		None
# Logic:		This function is a callback function which is called whenever the encoder detects a change in the angle. 
#               The function increments the global variable 'angle' by 18 degrees.
# Example Call:	This function is not called directly but is invoked by the encoder module in response to a change in angle.

def encoder_callback(channel):
    global angle
    angle += 18
#     print(angle)
    
# Function Name:measure_angle
# Input:		enc (integer) - the GPIO pin number to which the encoder is connected
# Output:		None
# Logic:		This function adds an event detection for the given encoder pin using the RPi.GPIO library. 
#               Whenever there is a change in the encoder state (i.e., the encoder disc is rotated), the encoder_callback function will be called. 
#               The callback function increments the global variable 'angle' by 18, which represents the resolution of the encoder.
# Example Call:	measure_angle(21)
def measure_angle(enc):
    GPIO.add_event_detect(enc, GPIO.BOTH, callback=encoder_callback, bouncetime=25)
    

# Function Name:forward
# Input:		lspeed (int), rspeed (int)
# Output:		None
# Logic:		This function sets the left and right motor speed and direction to move the robot forward
# Example Call:	forward(50, 50) # moves the robot forward at a speed of 50

def forward(lspeed,rspeed):
    pwm1.start(rspeed)
    pwm2.start(lspeed)
    GPIO.output(r1,0)
    GPIO.output(r2,1)
    GPIO.output(l1,0)
    GPIO.output(l2,1)

# Function Name:turn_inplace
# Input:		None
# Output:		None
# Logic:		This function controls the bot to rotate in its own position. It starts the PWM signals for the left and right wheels, 
#               sets the speed of the wheels to 70% of the maximum speed, and sets the direction of the wheels such that the bot rotates in its own position.
# Example Call:	turn_inplace()

def turn_inplace():
    pwm1.start(60)
    pwm2.start(60)
    GPIO.output(r1,1)
    GPIO.output(r2,0)
    GPIO.output(l1,0)
    GPIO.output(l2,1)
    
       
# Function Name:stop
# Input:		None
# Output:		None
# Logic:		Stops the movement of the robot by setting the speeds of both motors to 0
# Example Call:	stop()

def stop():
    print('STOP')
    forward(0,0)
    
# Function Name:left_turn
# Input:		None
# Output:		None
# Logic:		1. Sets global variables angle, cX, mid, image, thresh.
#               2. Prints 'LEFT'.
#               3. Initializes angle to 0.
#               4. Sleeps for 0.2 seconds.
#               5. Starts a new thread findAngle to call measure_angle function with right encoder as input.
#               6. Loop until angle is less than 300 degrees:
#                   a. Call forward function with 50 and 0 as inputs.
#               7. Call stop function.
#               8. Remove the event detection for right encoder.
#               9. Wait for findAngle thread to finish.
#               10. Reset cX to 0.
#               11. Loop until cX is less than 100:
#                   a. Call detect_line function with image and thresh as inputs.
#                   b. Call forward function with 40 and 0 as inputs.
#               12. Call stop function.
#               13. Print 'turned'.
# Example Call:	left_turn()

def left_turn():
    global angle, cX, mid, image, thresh
    print('LEFT')
    angle  = 0
    time.sleep(0.2)
    findAngle = Thread(target = measure_angle, args = [Renc])
    findAngle.start()
    
    while angle < 300:
        forward(50,0)
    stop()
    GPIO.remove_event_detect(Renc)
    findAngle.join()
    cX  = 0
    while cX < 100:
        #detect_line(image, thresh)
        forward(50,0)
    stop()
#     cX = 0
        
    
    print('turned')
     
# Function Name:right_turn
# Input:		None
# Output:		None
# Logic:		This function helps the bot to turn right by first detecting the current angle with the help of measure_angle() function,
#               then it makes the bot turn until it rotates 300 degrees. After that, it stops the rotation, and then it keeps moving forward
#               until the line is detected. Once the line is detected, it stops the bot's movement.
# Example Call:	right_turn()  
def right_turn():
    global angle, cX, mid, image, thresh
    print('RIGHT')
    angle  = 0
    time.sleep(0.2)
    findAngle = Thread(target = measure_angle, args = [Lenc])
    findAngle.start()
    
    while angle < 300:
        forward(0,50)
    stop()
    GPIO.remove_event_detect(Lenc)
    findAngle.join()
    cX = 200
    while cX > 380:
        #detect_line(image, thresh)
        forward(0,50)
    stop()
#     cX = 400
    print('turned')
        
# Function Name:detect_line
# Input:		image, thresh
# Output:		mid, image
# Logic:		This function receives an image and a threshold value as input.
#               It then applies thresholding to the image and applies the perspective transform
#               to obtain a bird's eye view. It then uses OpenCV's HoughLinesP function to detect lines in the image.
#               The function returns the midpoint of the detected lines and the transformed image.
# Example Call:	detect_line(image, thresh)

def reverse():
    global angle, cX, mid, image, thresh
    print('REVERSE')
    angle  = 0
    time.sleep(0.2)
    findAngle = Thread(target = measure_angle, args = [Lenc])
    findAngle.start()
    
    while angle < 400:
        turn_inplace()
    stop()
    GPIO.remove_event_detect(Lenc)
    findAngle.join()
    cX = 240
    while cX > 200:
        #detect_line(image, thresh)
        turn_inplace()
    stop()
    cX = 200
    print('turned')

    
   
# Function Name:wait
# Input:		None
# Output:		None
# Logic:		This function stops the motors of the robot and waits for 5 seconds before resuming operarions.
# Example Call: time,sleep(5)
def wait():
    stop()
    print('WAIT')
    time.sleep(5)

# Function Name:hsv_mask
# Input:		img (a BGR image)
# Output:		a masked image (a binary image where the color range defined by lower_hsv and higher_hsv is white and everything else is black)
# Logic:		convert the BGR image to HSV, define the color range to be extracted, create a mask for that color range, and apply the mask to 
#               the original image using bitwise_and operation. Finally, erode the image to remove noise.
# Example Call: hsv_mask(image)

def hsv_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([0, 100, 0])
    higher_hsv = np.array([42, 255, 255])
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

    frame = cv2.bitwise_and(img, img, mask=mask)
    kernel = np.ones((5,5),np.uint8)
    frame = cv2.erode(frame, kernel, iterations=1)

    return frame


# Function Name:process_img
# Input:		img - a single frame image from camera
# Output:		Returns two images-
#               1. thresholded image (after converting it to grayscale and applying thresholding)
#               2. HSV masked image (to extract only yellow color)
# Logic:		First, the image is converted to grayscale and then thresholding is applied to create binary image.
#               The binary image is then eroded to reduce noise.
#               Then, the image is masked using HSV color space to extract yellow color.
# Example Call: process_img(img)
def process_img(img):
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#         blur = cv2.blur(gray, (100,100))
# #         cv2.imshow('blur',blur)
        _ ,thresh=cv2.threshold(gray,90, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5),np.uint8)
        thresh = cv2.erode(thresh, kernel, iterations=1)
#         cv2.imshow('thresh',thresh)
        mask = hsv_mask(img)
#         cv2.imshow('mask',mask)
        return thresh, mask
        
# Function Name:detect_line
# Input:		img - input image, thresh - thresholded image
# Output:		None
# Logic:		This function detects the line in the thresholded image and draws it in the input image. 
#               The function also calculates the center of the line and stores it in cX and cY.
# Example Call:	detect_line(image, thresh)
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
            y2 = 143
            x2 = 0
            
       
        
        x3 = np.where(thresh[y3] == 0)[0][-1]
        
        y = np.where(thresh.T[239]==255)[0]
        if len(y)> 0:
            y4 = np.where(thresh.T[239]==255)[0][-1]
            x4 = np.where(thresh[y4] == 0)[0][-1]
        else:
            y4 = 143
            x4 = 239
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
#         cv2.line(img, (240,0), (240,143), (255,255,0), 2)
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

# Function Name:detect_node
# Input:		mask (binary image)
# Output:		True if node detected, False otherwise
# Logic:		- Convert white pixels to 1 and black pixels to 0
#               - Calculate the sum of all pixels in the mask
#               - If the sum is greater than 500, consider it as a node
#               - Return True if node detected, False otherwise
# Example Call:	detect_node(mask)
def detect_node(mask):
    mask = np.where(mask == 0 , mask , 1)
#     print(mask.sum())
    if mask.sum() > 500:
        return True
    return False
    
    
# Function Name:follow_line
# Input:		None
# Output:		None
# Logic:		This function follows the line using PID controller. The PID controller is used to adjust the speed of both motors according to
#               the error in position of the line. If the line is detected, it calculates the PID output and sets the speed of both motors based on the
#               output. If the line is not detected, the function tries to align the robot to the line. The function also updates the global variables rs, ls.
# Example Call:	follow_line()
def follow_line():
    global cX, mid, rs, ls, isLine
    try:
        pid = PID(cX,mid,angle)
        
        speed=40
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
#                 print('Line out right')
                rs = 0
                ls = 38
            else:
#                 print('Line out left')
                rs = 38
                ls = 0
            forward(rs, ls)
            
    except: pass

            

  
# Function Name:PID
# Input:		cX (integer) -current x coordinate of the center of the detected line 
#               mid(tuple of integer) - coordinates of the mid-point of the image
#               angle(float) - angle of the detected line
# Output:		pid (float) - calculated PID value
# Logic:		calculates the PID value for given current x coordinate of the center of the detected line, mid-point of the image
#               and the angle of the detected line. Proportional, Integral and Derivative values are calculated based on the difference between the current error and the previous error.
#               The PID value is then calculated by adding proportional, integral and derivative values multiplied by their respective constants.
# Example Call: PID(50, (160,120), 45.0)
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
#     print('errors:',error, d)
    pid = kp*error +kd*d
#     print('pid:',pid)
    
    return pid
##############################################################    

# Function Name:control_logic
# Input:		None
# Output:		None
# Logic:		Captures frames from the Raspberry Pi camera and processes them by calling the process_img function. The resulting
#               threshold and mask images are used to detect lines and nodes by calling the detect_line and detect_node functions.
#               The center of the detected line is calculated and used to call the follow_line function, which adjusts the
#               speed of the robot's motors to follow the line. Displays the resulting frame.
# Example Call: control_logic()
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
    print('camera started')
    camera = PiCamera()
    camera.resolution = (240, 144)
    camera.framerate = 45
    rawCapture = PiRGBArray(camera, size=(240, 144))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        mid = np.array(image.shape[:2])//2
        thresh, mask = process_img(image)
        
        detect_line(image, thresh)
#         print('follow')
       
        
        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF 
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        # if the `q` key was pressed, break from the loop
        if key == ord("q") or END:
            stop()
            return

    ##########################################################

# Function Name:move_bot
# Input:		path(list) - a list of directions to be followed
# Output:		None
# Logic:		This function takes a list of directions as input and then moves the bot accordingly.
#               It uses the 'detect_node' function to detect the nodes in the path and then decides on the actions
#               to be taken accordingly. It also uses functions like 'follow_line', 'stop', 'right_turn', 'left_turn', 'wait' and
#               'reverse' to move the bot in different directions. The function 'measure_angle' is used to calculate the angle
#               at which the bot is turned. The function continues to run until all the directions have been executed or
#               the 'END' flag has been set to True.
# Example Call: move_bot(path) - where path = ['STRAIGHT', 'RIGHT', 'LEFT', 'WAIT_5', 'REVERSE', 'STRAIGHT'] is a list of directions to be followed.

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
    print('ended node')
    
    counter+=1
    print('starting')
    while True:
        if not detect_node(mask):
                #detect_line(image, thresh)
                follow_line()

        else:
#                 stop()
                
#                     END = True
                    
                while detect_node(mask):
                    forward(50,50)
#                     forward(100,100)
                stop()
                print('node')
#                 print("h")
                angle  = 0
                findAngle = Thread(target = measure_angle, args = [Renc])
                findAngle.start()
                
                while angle < 200:
                    forward(50,50)
#                     forward(100,100)

                stop()
                print("h")
                if counter >= len(path):
                    stop()
                    return
                directions[path[counter]]()
                time.sleep(1)
                print('ended node')
                
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
    
    path = ['REVERSE','STRAIGHT','STRAIGHT','LEFT']
    line_follow = Thread(target = move_bot, args = [path])
    image_process.start()
    time.sleep(2)
    line_follow.start()

    ##########################################################






