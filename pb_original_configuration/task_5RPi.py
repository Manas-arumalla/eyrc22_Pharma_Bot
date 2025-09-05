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

# Team ID:			PB_1996
# Author List:		VYBHAV RAGAVENDRA DEVARAKONDA, MANAS REDDY.A, VIGNESH.M, GNANESH.K
# Filename:			task_6.py
# Theme:			PHARMABOT
# Functions:		setup_client(host, port),receive_message_via_socket(client),send_message_via_socket(client),
#                   message),rgb_led_setup(R,G,B),rgb_led_set_color(rgb_index,color),rgb_off(rgb_index),followLine(message)
# Global variables:	R = G = B = NONE
####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv)                    ##
##############################################################
import socket
import linefollower_test
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
# Function Name:setup_client
# Input:		host (string), port (string)
# Output:       client (socket object)
# Logic:        This function creates a new socket client using the socket library.
#               The function tries to connect to a socket server using the host and port specified.
#               The function returns the client socket object if the connection is successful, otherwise it returns None.
# Example Call: client = setup_client(host, port)
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
# Function Name:receive_message_via_socket
# Input:		client [socket object] - client socket object created by setup_client() function
# Output:       message [string] - message received through socket communication
# Logic:        This function listens for a message from the specified socket connection and returns the message when received.
#               The function receives the message in a byte format using the recv() method, and decodes it to a string format
#               using the decode() method.
# Example Call: message = receive_message_via_socket(connection)

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

# Function Name:send_message_via_socket
# Input:		client - client socket object created by setup_client() function
#               message - message sent through socket communication
# Output:       None
# Logic:        This function encodes the given message and sends it over the specified socket connection.
# Example Call: send_message_via_socket(connection, message)
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
# Function Name:rgb_led_setup
# Input:		R,G,B (GPIO pin numbers for Red, Green and Blue pins of RGB LED)
# Output:       None
# Logic:        This function configures GPIO pins connected to the RGB LED as output and enables PWM on the pins.
#               It sets up the PWM for the RGB LED pins, and appends the PWM objects to the lrgb list.
# Example Call: To set up the RGB LED with pins 4, 5, and 6 (BCM notation):rgb_led_setup(4, 5, 6)
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
# Function Name:rgb_led_set_color
# Input:		color (string)
# Output:		None
# Logic:		The function receives a string color and an index number for the rgb led. The function then
#               retrieves the pwm values of the color from a dictionary and converts them to a range from 0 to 100.
#               It then iterates over the range 0 to 3 and starts the pwm values of the corresponding pins using
#               the index of the rgb led. This sets the color of the rgb led to the received color.
# Example Call: rgb_led_set_color(color)
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
# Function Name:rgb_off
# Input:		rgb_index : [int]  the index of the RGB LED whose color has to be turned off
# Output:		None
# Logic:		This function turns off the color of the specified RGB LED by setting the PWM value of each of the three pins to 0.
# Example Call:	rgb_off(0)
def rgb_off(rgb_index):

    for i in range(3):
        lrgb[rgb_index][i].start(0)
     
# Function Name:followLine
# Input:		message [string]
# Output:		
# Logic:		- Split the message using comma as delimiter and get the list of path coordinates
#               - Start a new thread to call move_bot function of linefollower_test module with path coordinates as argument
#               - Wait for 2 seconds to allow the thread to start
#               - Start the thread
#               - Wait for the thread to complete using join() method
#               - Send "Reached" message to server using send_message_via_socket() function
#               - Wait for 1 second
#               - Return None
# Example Call:	followLine("1,1 2,2 3,3,")
def followLine(message):
        path = message.split(',')[:-1]
        print(path)
        
        line_follow = Thread(target = linefollower_test.move_bot, args = [path])
        time.sleep(2)
        line_follow.start()
        line_follow.join()
#         send_message_via_socket(client, 'Reached')
        time.sleep(1)

     
     



    ##########################################################

if __name__ == "__main__":

        host = "192.168.135.235"
        port = 5050

        

        pins={"redPin3":17,"greenPin3":5,"bluePin3":19,"redPin2":16,"greenPin2":25,"bluePin2":24, 'redPin1':27, 'greenPin1':23, "bluePin1":22 } # add the other pins
        lrgb=[]

        
        
        

        ## PWM values to be set for rgb led to display different colors
        pwm_values = {"White":(255,255,255),"Red": (255, 0, 0), "Blue": (0, 0, 255), "Green": (0, 255, 0), "Orange": (255, 35, 0), "Pink": (255, 0, 122), "Skyblue": (0, 100, 100)}


        ## Configure rgb led pins
        rgb_led_setup(pins["redPin1"],pins["greenPin1"],pins["bluePin1"])
        rgb_led_setup(pins["redPin2"],pins["greenPin2"],pins["bluePin2"])
        rgb_led_setup(pins["redPin3"],pins["greenPin3"],pins["bluePin3"])
        rgb_off(0)
        rgb_off(1)
        rgb_off(2)
        image_process = Thread(target = linefollower_test.control_logic)
        image_process.start()



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
                if message!="stop":
                    followLine(message)
                
                
                time.sleep(2)
                send_message_via_socket(client, 'Reached')
                picked = receive_message_via_socket(client)
                
                print(".")
                print(picked)
                rgb_led_set_color(i, picked)
                time.sleep(2)
                send_message_via_socket(client, 'RGB_ON')
                print(i,picked)

            length = receive_message_via_socket(client)
            send_message_via_socket(client, 'hi')
            print(length)

            for i in range(int(length)):

                # have to edit this part to get the delivery path
                message = receive_message_via_socket(client)
                print(message)
                followLine(message)
                send_message_via_socket(client, 'Reached')
                index = receive_message_via_socket(client)
                print(index)
                rgb_off(int(index))
                send_message_via_socket(client, 'RGB_OFF')

        message = receive_message_via_socket(client)
        print(message)
        followLine(message)
        send_message_via_socket(client, 'End of Task')
        rgb_led_set_color(0, "White")
        rgb_led_set_color(1, "White")
        rgb_led_set_color(2, "White")
        linefollower_test.END = True
        image_process.join()
        sleep(10)

            
        
        

