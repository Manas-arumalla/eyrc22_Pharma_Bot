'''
*****************************************************************************************
*
*        		===============================================
*           		Pharma Bot (PB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 3D of Pharma Bot (PB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*d
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			socket_client_rgb.py
# Functions:		
# 					[ Comma separated list of functions in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv)                    ##
##############################################################
import socket
import linefollower
import time
import os, sys
import RPi.GPIO as GPIO
from threading import Thread
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

R = G = B = None
#################################x#############################

def setup_client(host, port):
    

    """
    Purpose:
    ---
    This function creates a new socket client and then tries
    to connect to a socket server.

    Input Arguments:
    ---
    `host` :	[ string ]
            host name or ip address for the server

    `port` : [ string ]
            integer value specifying port name
    Returns:

    `client` : [ socket object ]
               a new client socket object
    ---

    
    Example call:
    ---
    client = setup_client(host, port)
    """ 

    client = None

    ##################	ADD YOUR CODE HERE	##################
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client.connect((host, port))


    ##########################################################

    return client

def receive_message_via_socket(client):
    """
    Purpose:
    ---
    This function listens for a message from the specified
    socket connection and returns the message when received.

    Input Arguments:
    ---
    `client` :	[ socket object ]
            client socket object created by setup_client() function
    Returns:
    ---
    `message` : [ string ]
            message received through socket communication
    
    Example call:
    ---
    message = receive_message_via_socket(connection)
    """

    message = None

    ##################	ADD YOUR CODE HERE	##################
    data = client.recv(1024)
    message = data.decode()

    ##########################################################

    return message

def send_message_via_socket(client, message):
    """
    Purpose:
    ---
    This function sends a message over the specified socket connection

    Input Arguments:
    ---
    `client` :	[ socket object ]
            client socket object created by setup_client() function

    `message` : [ string ]
            message sent through socket communication

    Returns:
    ---
    None
    
    Example call:
    ---
    send_message_via_socket(connection, message)
    """

    ##################	ADD YOUR CODE HERE	##################
    data = message.encode()

    client.sendall(data)

    ##########################################################

def rgb_led_setup(R,G,B):
    """
    Purpose:
    ---
    This function configures pins connected to rgb led as output and
    enables PWM on the pins 

    Input Arguments:
    ---
    You are free to define input arguments for this function.

    Returns:
    ---
    You are free to define output parameters for this function.
    
    Example call:
    ---
    rgb_led_setup()
    """

    ##################	ADD YOUR CODE HERE	##################

    GPIO.setup(R,GPIO.OUT)
    GPIO.setup(B,GPIO.OUT)
    GPIO.setup(G,GPIO.OUT)
#     GPIO.setup(gndPin, GPIO.OUT)
#     
#     GPIO.output(gndPin, GPIO.LOW)
    Red = GPIO.PWM(R, 2000)
    Green = GPIO.PWM(G, 2000)
    Blue = GPIO.PWM(B, 2000)
    lrgb.append([Red, Green, Blue])




    ##########################################################
    
def rgb_led_set_color(rgb_index,color):
    """
    Purpose:    
    ---
    This function takes the color as input and changes the color of rgb led
    connected to Raspberry Pi 

    Input Arguments:
    ---

    `color` : [ string ]
            color detected in QR code communicated by server
    
    You are free to define any additional input arguments for this function.

    Returns:
    ---
    You are free to define output parameters for this function.
    
    Example call:
    ---
    rgb_led_set_color(color)
    """    

    ##################	ADD YOUR CODE HERE	##################
    rgb = [i*(100/255) for i in pwm_values[color]]
    
    for i in range(3):
        lrgb[rgb_index][i].start(rgb[i])

def rgb_off(rgb_index):

    for i in range(3):
        lrgb[rgb_index][i].start(0)
     

def followLine(message):
        path = message.split(',')[:-1]
        print(path)
        image_process = Thread(target = linefollower.control_logic)
        image_process.start()
        line_follow = Thread(target = linefollower.move_bot, args = [path])
        time.sleep(2)
        line_follow.start()
        line_follow.join()
        send_message_via_socket(client, 'Reached')
        time.sleep(1)

     
     



    ##########################################################

if __name__ == "__main__":

        host = "192.168.141.35"
        port = 5050

        

        pins={"redPin1":17,"greenPin1":5,"bluePin1":19,"redPin2":16,"greenPin2":25,"bluePin2":24, 'redPin3':27, 'greenPin3':23, "bluePin3":22 } # add the other pins
        lrgb=[]

        
        
        

        ## PWM values to be set for rgb led to display different colors
        pwm_values = {"Red": (255, 0, 0), "Blue": (0, 0, 255), "Green": (0, 255, 0), "Orange": (255, 35, 0), "Pink": (255, 0, 122), "Skyblue": (0, 100, 100)}


        ## Configure rgb led pins
        rgb_led_setup(pins["redPin1"],pins["greenPin1"],pins["bluePin1"])
        rgb_led_setup(pins["redPin2"],pins["greenPin2"],pins["bluePin2"])
        rgb_led_setup(pins["redPin3"],pins["greenPin3"],pins["bluePin3"])



        ## Set up new socket client and connect to a socket server
        try:
            client = setup_client(host, port)


        except socket.error as error:
            print("Error in setting up server")
            print(error)
            sys.exit()

        ## Wait for START command from socket_server_rgb.py
        
    
        counter = receive_message_via_socket(client)
        print(".")
        for count in range(int(counter)):

            # Pick Up
            
            for i in range(3):
                message = receive_message_via_socket(client)
                print(message)
                if message=='b':
                    break
#                 if message!='stop':
#                     followLine(message)
                
                
                time.sleep(5)
                send_message_via_socket(client, 'Reached')
                picked = receive_message_via_socket(client)
                
                print(".")
                print(picked)
                rgb_led_set_color(i, picked)
                send_message_via_socket(client, 'RGB_ON')
                print(i,picked)

            length = receive_message_via_socket(client)
            send_message_via_socket(client, 'fuck')
            print(length)

            for i in range(int(length)):

                # have to edit this part to get the delivery path
                # message = receive_message_via_socket(client)
                # followLine(message)
                index = receive_message_via_socket(client)
                print(".")
                rgb_off(int(index))
                send_message_via_socket(client, 'RGB_OFF')


        
        
        

