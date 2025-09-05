'''
*****************************************************************************************
*
*        		===============================================
*           		Pharma Bot (PB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 3C of Pharma Bot (PB) Theme (eYRC 2022-23).
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
# Filename:			task_3c.py
# Functions:		[ perspective_transform, transform_values, set_values ]
# 					


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the five available  ##
## modules for this task                                    ##
##############################################################
import cv2 
import numpy 
from zmqRemoteApi import RemoteAPIClient
import zmq
##############################################################

#################################  ADD UTILITY FUNCTIONS HERE  #######################
    
#####################################################################################

task_1b = __import__('task_1b')
def perspective_transform(image, ArUco_details_dict, ArUco_corners):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns the image after 
    applying perspective transform on it. Using this function, you should
    crop out the arena from the full frame you are receiving from the 
    overhead camera feed.

    HINT:
    Use the ArUco markers placed on four corner points of the arena in order
    to crop out the required portion of the image.

    Input Arguments:
    ---
    `image` :	[ numpy array ]
            numpy array of image returned by cv2 library 

    Returns:
    ---
    `warped_image` : [ numpy array ]
            return cropped arena image as a numpy array
    
    Example call:
    ---
    warped_image = perspective_transform(image)
    """   
    warped_image = [] 
#################################  ADD YOUR CODE HERE  ###############################
    # ArUco_details_dict, ArUco_corners  = task_1b.detect_ArUco_details(image)

    image_cp = numpy.copy(image)
    task_1b.mark_ArUco_image(image_cp, ArUco_details_dict, ArUco_corners)
    cv2.imshow('img_cp', image_cp)

    # pts = numpy.float32([numpy.array(ArUco_details_dict[1][0])+[10,10],numpy.array(ArUco_details_dict[2][0])+ [-10,10], numpy.array(ArUco_details_dict[3][0]) + [-10,-10], numpy.array(ArUco_details_dict[4][0]) + [10,-10]])
    pts = numpy.float32([numpy.array(ArUco_details_dict[1][0]),numpy.array(ArUco_details_dict[2][0]), numpy.array(ArUco_details_dict[3][0]), numpy.array(ArUco_details_dict[4][0])])
    pts2 = numpy.float32([[512,512],[0,512], [0,0], [512,0]])
    mat = cv2.getPerspectiveTransform(pts, pts2)
    warped_image = cv2.warpPerspective(image, mat, [512,512])

    cv2.imshow('img', warped_image)
    
######################################################################################

    return warped_image

def transform_values(image):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns the 
    position and orientation of the ArUco marker (with id 5), in the 
    CoppeliaSim scene.

    Input Arguments:
    ---
    `image` :	[ numpy array ]
            numpy array of image returned by camera

    Returns:
    ---
    `scene_parameters` : [ list ]
            a list containing the position and orientation of ArUco 5
            scene_parameters = [c_x, c_y, c_angle] where
            c_x is the transformed x co-ordinate [float]
            c_y is the transformed y co-ordinate [float]
            c_angle is the transformed angle [angle]
    
    HINT:
        Initially the image should be cropped using perspective transform 
        and then values of ArUco (5) should be transformed to CoppeliaSim
        scale.
    
    Example call:
    ---
    scene_parameters = transform_values(image)
    """   
    
    global la , scene_parameters
    scene_parameters = []
#################################  ADD YOUR CODE HERE  ###############################
    # try:
    # cv2.imshow('scene', image)
    ArUco_details_dict, ArUco_corners  = task_1b.detect_ArUco_details(image)
    if 5 in ArUco_details_dict.keys():
        x,y = ArUco_details_dict[5][0]
        transform_const = 1.91/512
        x1,y1 = x*transform_const, y*transform_const

        a= ArUco_details_dict[5][1]
        print(a)
        if a >0:
            a = -(180-a)
        elif a < 0 :
            a = (180+a)
        else:
            a = 180
        print(a)

        a = a*numpy.pi/180


        scene_parameters = [[0.955 - x1, y1 - 0.955, 0.0325], [a - la, 0, 0]] #
        la = a
        # print(scene_parameters) 
    else:
        print('Missing')
    # except: 
    #     print('error')

######################################################################################

    return scene_parameters


def set_values(sim,scene_parameters):
    """
    Purpose:
    ---
    This function takes the scene_parameters, i.e. the transformed values for
    position and orientation of the ArUco marker, and sets the position and 
    orientation in the CoppeliaSim scene.

    Input Arguments:
    ---
    `scene_parameters` :	[ list ]
            list of co-ordinates and orientation obtained from transform_values()
            function

    Returns:
    ---
    None

    HINT:
        Refer Regular API References of CoppeliaSim to find out functions that can
        set the position and orientation of an object.
    
    Example call:
    ---
    set_values(scene_parameters)
    """   
    aruco_handle = sim.getObject('/alphabot')
    arena_handle = sim.getObject('/Arena')
    print('SP:', scene_parameters)
#################################  ADD YOUR CODE HERE  ###############################

    sim.setObjectPosition(aruco_handle,sim.handle_world ,scene_parameters[0])
    sim.setObjectOrientation(aruco_handle,aruco_handle,scene_parameters[1])
    # sim.setObjectOrientation(alphabot,sim.handle_world,[-90,0,90])
######################################################################################

    return None

def main():
    global la
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    # alphabot = sim.getObject('/alphabot')
    # sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
    # sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
    
    
#################################  ADD YOUR CODE HERE  ################################
    
    

    cam = cv2.VideoCapture('http://192.168.1.14:8080/video') # change the camera option accordingly
    alphabot = sim.getObject('/alphabot')
    sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
    sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])

    #detect all aruco initially
    while True: 
        ret, frame = cam.read()
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('frame', frame)
        cv2.waitKey(1) 
        
        try:
            ArUco_details_dict, ArUco_corners  = task_1b.detect_ArUco_details(frame)
            image_cp = numpy.copy(frame)
            task_1b.mark_ArUco_image(image_cp, ArUco_details_dict, ArUco_corners)
            cv2.imshow('img_cp', image_cp)

            if len(ArUco_details_dict) == 5:
                break

        except :
            print('Detecting All arucos')


    la = 179
    while True: 
        ret, frame = cam.read()
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
        
        
        try:
            Tframe = perspective_transform(frame, ArUco_details_dict, ArUco_corners)  
            scene_parameters = transform_values(Tframe)
            set_values(sim,scene_parameters)

        except Exception as e:
            # print(e)
            pass

        # cv2.imshow('aruco', frame)  
        
        cv2.waitKey(1)  

    cv2.imshow('frame', frame)


#######################################################################################


if __name__ == '__main__':
    
    main()

    
