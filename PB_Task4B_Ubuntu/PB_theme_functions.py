'''
*****************************************************************************************
*
*        		     ===============================================
*           		       Pharma Bot (PB) Theme (eYRC 2022-23)
*        		     ===============================================
*
*  This script contains all the past implemented functions of Pharma Bot (PB) Theme 
*  (eYRC 2022-23).
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			PB_theme_functions.py
# Functions:		
# 					[ Comma separated list of functions in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv)                    ##
##############################################################
import socket
import time
import os, sys
from zmqRemoteApi import RemoteAPIClient
import traceback
import zmq
import numpy as np
import cv2
from pyzbar.pyzbar import decode
import json
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
b = [255,0,0]
g = [0,255,0]
r = [0,0,255]
w = [255,255,255]
p = [189, 43,105]

nodes = {}

##############################################################


################## ADD SOCKET COMMUNICATION ##################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 3D for setting up a Socket
Communication Server in this section
"""

def setup_server(host, port):

	"""
	Purpose:
	---
	This function creates a new socket server and then binds it 
	to a host and port specified by user.

	Input Arguments:
	---
	`host` :	[ string ]
			host name or ip address for the server

	`port` : [ int ]
			integer value specifying port number
	Returns:

	`server` : [ socket object ]
	---

	
	Example call:
	---
	server = setup_server(host, port)
	""" 

	server = None

	##################	ADD YOUR CODE HERE	##################
	
	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	server.bind((host, port))
	##########################################################

	return server

def setup_connection(server):
	"""
	Purpose:
	---
	This function listens for an incoming socket client and
	accepts the connection request

	Input Arguments:
	---
	`server` :	[ socket object ]
			socket object created by setupServer() function
	Returns:
	---
	`connection` : [ socket object ]
	        socket connection object

	`address` : [ tuple ]
	        address of socket connection
	
	Example call:
	---
	connection, address = setup_connection(server)
	"""
	connection = None
	address = None

	##################	ADD YOUR CODE HERE	##################
	server.listen()

	connection, address = server.accept()

	##########################################################

	return connection, address

def receive_message_via_socket(connection):
	"""
	Purpose:
	---
	This function listens for a message from the specified
	socket connection and returns the message when received.

	Input Arguments:
	---
	`connection` :	[ connection object ]
			connection object created by setupConnection() function
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


	data = connection.recv(1024)
	message = data.decode()
	##########################################################

	return message

def send_message_via_socket(connection, message):
	"""
	Purpose:
	---
	This function sends a message over the specified socket connection

	Input Arguments:
	---
	`connection` :	[ connection object ]
			connection object created by setupConnection() function

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

	connection.sendall(data)

	##########################################################


##############################################################
##############################################################

######################### ADD TASK 2B ########################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 2B for reading QR code from
CoppeliaSim arena in this section
"""



def getVision(sim):
	vs = sim.getObject('/vision_sensor')

	img, rX, rY = sim.getVisionSensorCharImage(vs)
	img = np.frombuffer(img, dtype = np.uint8).reshape(rX,rY,3)
	img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gray = cv2.blur(gray, (50,100))
	# cv2.imshow('blur', gray)
	_, gray = cv2.threshold(gray, 225, 255, cv2.THRESH_BINARY_INV)
	# cv2.imshow('thresh',gray)

	return img, gray


def read_qr_code(sim):
	"""
	Purpose:
	---
	This function detects the QR code present in the CoppeliaSim vision sensor's 
	field of view and returns the message encoded into it.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	`qr_message`   :    [ string ]
		QR message retrieved from reading QR code

	Example call:
	---
	control_logic(sim)
	"""
	qr_message = None
	
	##############  ADD YOUR CODE HERE  ##############

	img, gray  = getVision(sim)
	m = decode(img)[0][0]
	# print(m, type(m))
	qr_message = json.loads(m.decode('utf-8'))
	cv2.waitKey(5)

	##################################################

	return qr_message





def activateQr(sim, checkpoint):
	# # Retrieve the handle of the Arena_dummy scene object.
	qr_code = sim.getObject("/Arena/qr_plane") 

	# ## Retrieve the handle of the child script attached to the Arena_dummy scene object.
	# childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")

	# ## Call the activate_qr_code() function defined in the child script to make the QR code visible at checkpoint E
	# sim.callScriptFunction("activate_qr_code", childscript_handle, checkpoint)

	sim.setObjectInt32Param(qr_code, sim.objintparam_visibility_layer, 1)




def deactivateQr(sim, checkpoint):
	## Retrieve the handle of the Arena_dummy scene object.
	qr_code = sim.getObject("/Arena/qr_plane") 

	# ## Retrieve the handle of the child script attached to the Arena_dummy scene object.
	# childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")

	# ## Call the activate_qr_code() function defined in the child script to make the QR code visible at checkpoint E
	# sim.callScriptFunction("deactivate_qr_code", childscript_handle, checkpoint)

	sim.setObjectInt32Param(qr_code, sim.objintparam_visibility_layer, 0)


def deliverPackage(sim, checkpoint, package):
	## Retrieve the handle of the Arena_dummy scene object.
	arena_dummy_handle = sim.getObject("/Arena") 

	# ## Retrieve the handle of the child script attached to the Arena_dummy scene object.
	# childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")

	# ## Deliver package_1 at checkpoint E
	# sim.callScriptFunction("deliver_package", childscript_handle, package, checkpoint)

##############################################################
##############################################################

############### ADD ARENA PARAMETER DETECTION ################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 1A and 3A for detecting arena parameters
from configuration image in this section
"""


##############################################################
##############################################################

def detect_nodes(maze_image):
	# cv2.imshow('image', maze_image)
	imgray=cv2.cvtColor(maze_image,cv2.COLOR_BGR2GRAY)
	_, thresh = cv2.threshold(imgray, 240, 255, cv2.THRESH_BINARY_INV )
	# cv2.imshow('thresh', thresh)
	img1 = np.zeros(maze_image.shape, np.uint8)


	X,Y = np.where(np.all(maze_image == b, axis=2))
	img1[X,Y] = w
	X,Y = np.where(np.all(maze_image == r, axis=2))
	img1[X,Y] = w 
	X,Y = np.where(np.all(maze_image == p, axis=2))
	img1[X,Y] = w 

	coords = np.where(np.all(maze_image == g, axis=2))
	X = []
	Y = []
	for i,y in enumerate(coords[0]):
		if y > 400:
			Y.append(coords[1][i])
			X.append(coords[0][i])
	img1[X,Y] = w



	gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

	contours, h = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	cv2.drawContours(img1, contours , -1  ,color=[0,0,255])

	mids = []
	for cnt in contours:
		mids.append((cnt[0][0] + cnt[2][0])//2)
	# cv2.imshow('nodes', img1)

	nodes = {}
	j = 35
	for i in range(1,7):
		for alpha in 'ABCDEF':
			nodes[alpha+str(i)] = mids[j]
			j-=1

	# cv2.waitKey(0)
	return nodes




##############################################################


def detect_traffic_signals(maze_image, nodes):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list of
	nodes in which traffic signals are present in the image

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`traffic_signals` : [ list ]
			list containing nodes in which traffic signals are present
	
	Example call:
	---
	traffic_signals = detect_traffic_signals(maze_image)
	"""    
	traffic_signals = []

	##############	ADD YOUR CODE HERE	##############
	
	for node, point in nodes.items():

		if np.array_equal(maze_image[point[1],point[0]], np.array(r)):
			traffic_signals.append(node)
	traffic_signals = sorted(traffic_signals)
	##################################################
	
	return traffic_signals
##############################################################

def detect_start_node(maze_image,nodes):


	##############	ADD YOUR CODE HERE	##############
	
	for node, point in nodes.items():

		if np.array_equal(maze_image[point[1],point[0]], g):
			start_node = node
	##################################################
	
	return start_node


def detect_end_node(maze_image,nodes):

	##############	ADD YOUR CODE HERE	##############
	
	for node, point in nodes.items():

		if np.array_equal(maze_image[point[1],point[0]], p):
			end_node = node
	##################################################
	
	return end_node


def detect_horizontal_roads_under_construction(maze_image , nodes):
	
	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list
	containing the missing horizontal links

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`horizontal_roads_under_construction` : [ list ]
			list containing missing horizontal links
	
	Example call:
	---
	horizontal_roads_under_construction = detect_horizontal_roads_under_construction(maze_image)
	"""    
	horizontal_roads_under_construction = []

	##############	ADD YOUR CODE HERE	##############
	for i in range(1,7):
		before = 'A'
		for alpha in 'BCDEF':
			m = (nodes[alpha+str(i)] + nodes[before+str(i)])//2
			
			if np.array_equal(maze_image[m[1], m[0]] , w):
				horizontal_roads_under_construction.append([before+str(i) , alpha+str(i)])

			before = alpha


	horizontal_roads_under_construction = sorted(horizontal_roads_under_construction)
	# print(horizontal_roads_under_construction)

	##################################################
	
	return horizontal_roads_under_construction

def detect_vertical_roads_under_construction(maze_image, nodes):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list
	containing the missing vertical links

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`vertical_roads_under_construction` : [ list ]
			list containing missing vertical links
	
	Example call:
	---
	vertical_roads_under_construction = detect_vertical_roads_under_construction(maze_image)
	"""    
	vertical_roads_under_construction = []

	##############	ADD YOUR CODE HERE	##############

	
	for alpha in 'ABCDEF':
		before = 1
		for i in range(2,7):

			m = (nodes[alpha+str(i)] + nodes[alpha+str(before)])//2
			if np.array_equal(maze_image[m[1], m[0]] , w):
				vertical_roads_under_construction.append([alpha+str(before) , alpha+str(i)])

			before = i


	vertical_roads_under_construction = sorted(vertical_roads_under_construction)
	# print(vertical_roads_under_construction)

	##################################################
	
	return vertical_roads_under_construction


##############################################################
	
def detect_all_nodes(maze_image):
	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list of
	nodes in which traffic signals, start_node and end_node are present in the image

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`traffic_signals, start_node, end_node` : [ list ], str, str
			list containing nodes in which traffic signals are present, start and end node too
	
	Example call:
	---
	traffic_signals, start_node, end_node = detect_all_nodes(maze_image)
	"""    

	global nodes
	traffic_signals = []
	start_node = ""
	end_node = ""

	##############	ADD YOUR CODE HERE	##############

	nodes = detect_nodes(maze_image)

	traffic_signals = detect_traffic_signals(maze_image, nodes)
	start_node = detect_start_node(maze_image, nodes)
	end_node = detect_end_node(maze_image, nodes)
	##################################################

	return traffic_signals, start_node, end_node


def detect_medicine_packages(maze_image):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a nested list of
	details of the medicine packages placed in different shops

	** Please note that the shop packages should be sorted in the ASCENDING order of shop numbers 
	   as well as in the alphabetical order of colors.
	   For example, the list should first have the packages of shop_1 listed. 
	   For the shop_1 packages, the packages should be sorted in the alphabetical order of color ie Green, Orange, Pink and Skyblue.

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`medicine_packages` : [ list ]
			nested list containing details of the medicine packages present.
			Each element of this list will contain 
			- Shop number as Shop_n
			- Color of the package as a string
			- Shape of the package as a string
			- Centroid co-ordinates of the package
	Example call:
	---
	medicine_packages = detect_medicine_packages(maze_image)
	"""    
	medicine_packages = []
	

	##############	ADD YOUR CODE HERE	##############
	nodes = detect_nodes(maze_image)
	before = 'A'
	i = 1
	shop_img={}
	for alpha in "BCDEF":
		shop_img['Shop_'+str(i)]= maze_image[ nodes[before+str(1)][1]:nodes[alpha+str(2)][1] ,nodes[before+str(1)][0] : nodes[alpha+str(2)][0] ]
		before = alpha
		i+=1

	y = 100
	x = 100
	for shop,images in shop_img.items():

		images.astype('int32')

		b=cv2.cvtColor(shop_img[shop],cv2.COLOR_BGR2GRAY)
		_, b = cv2.threshold(b, 240, 255, cv2.THRESH_BINARY_INV )
		contours,hierarchy=cv2.findContours(b,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		######## Draw contours ###########
		# cv2.drawContours(shop_img[shop] ,contours , -1 , [0,255,0] )
		# cv2.imshow(shop , shop_img[shop])

		#print(contours , end = '\nnext\n')
		for cnt in contours:
			area=cv2.contourArea(cnt)
			shape = '' 
			col = ''
			if area<500:

				if len(cnt) == 4:
					shape= 'cube'
				elif len(cnt)==39:
					shape = 'cone'
				elif len(cnt) == 36:
					shape = 'cylinder'
				# print(shape)
				m=cv2.moments(cnt)
				
				if m["m00"] !=0:
					cx=int(m["m10"]/m["m00"])
					cy=int(m["m01"]/m["m00"])
				else:
					cx,cy=0,0
				if np.array_equal(shop_img[shop][cy,cx], g):
					col = 'Green'
				elif np.array_equal(shop_img[shop][cy,cx], [255,255,0]) :
					col = 'Skyblue'
				elif np.array_equal(shop_img[shop][cy,cx], [0,127,255]) :
					col = 'Orange'
				elif np.array_equal(shop_img[shop][cy,cx], [180,0,255]) :
					col = 'Pink'
				# print(col)

				medicine_packages.append([shop , col, shape, [cx+x,cy+y]])
		x+=100
				
				########## to draw and confirm the shape details ########
				# cv2.circle(shop_img[shop],(cx,cy),2,(0,0,255),-1)
				# cv2.putText(shop_img[shop],shape,(cx,cy),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0),1)
				# cv2.imshow(shop,shop_img[shop])


	medicine_packages = sorted(medicine_packages)        
	# print(medicine_packages)

	##################################################

	return medicine_packages

def detect_arena_parameters(maze_image):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a dictionary
	containing the details of the different arena parameters in that image

	The arena parameters are of four categories:
	i) traffic_signals : list of nodes having a traffic signal
	ii) horizontal_roads_under_construction : list of missing horizontal links
	iii) vertical_roads_under_construction : list of missing vertical links
	iv) medicine_packages : list containing details of medicine packages

	These four categories constitute the four keys of the dictionary

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`arena_parameters` : { dictionary }
			dictionary containing details of the arena parameters
	
	Example call:
	---
	arena_parameters = detect_arena_parameters(maze_image)
	"""    
	arena_parameters = {}

	##############	ADD YOUR CODE HERE	##############

	traffic_signals, start_node, end_node = detect_all_nodes(maze_image)
	nodes = detect_nodes(maze_image)
	arena_parameters['traffic_signals'] = traffic_signals
	arena_parameters['start_node'] = start_node
	arena_parameters['end_node'] = end_node
	arena_parameters['horizontal_roads_under_construction'] = detect_horizontal_roads_under_construction(maze_image, nodes)
	arena_parameters['vertical_roads_under_construction'] = detect_vertical_roads_under_construction(maze_image, nodes)
	arena_parameters['medicine_packages'] = detect_medicine_packages(maze_image)
	arena_parameters['paths']  = detect_paths_to_graph(maze_image, nodes)
	# print(arena_parameters)
	##################################################
	
	return arena_parameters



def detect_paths_to_graph(maze_image, nodes):
	"""
	Purpose:
	---
	This function takes the image as an argument and returns a dictionary of the
	connect path from a node to other nodes and will be used for path planning

	HINT: Check for the road besides the nodes for connectivity 

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`paths` : { dictionary }
			Every node's connection to other node and set it's value as edge value 
			Eg. : { "D3":{"C3":1, "E3":1, "D2":1, "D4":1}, 
					"D5":{"C5":1, "D2":1, "D6":1 }  }

			Why edge value 1? -->> since every road is equal

	Example call:
	---
	paths = detect_paths_to_graph(maze_image)
	"""    

	paths = {}

	##############	ADD YOUR CODE HERE	##############
	# commented lines are for set results (previous version)

	alphabets = 'ABCDEF'
	for k , alpha in enumerate(alphabets):
		for i in range(1,7):
			paths[alpha+str(i)] = {} 								#set()
			if alpha != 'A':
				paths[alpha+str(i)][alphabets[k-1]+str(i)] = 1 		#paths[alpha+str(i)].add(alphabets[k-1]+str(i))
			if alpha !='F':
				paths[alpha+str(i)][alphabets[k+1]+str(i)] = 1 		#paths[alpha+str(i)].add(alphabets[k+1]+str(i))
			if i != 1:
				paths[alpha+str(i)][alpha+str(i-1)] = 1 			#paths[alpha+str(i)].add(alpha+str(i-1))
			if i != 6:
				paths[alpha+str(i)][alpha+str(i+1)] = 1 			#paths[alpha+str(i)].add(alpha+str(i+1))
	
	vertical_roads_under_construction = [ set(road) for road in detect_vertical_roads_under_construction(maze_image, nodes)]
	horizontal_roads_under_construction = [set(road) for road in detect_horizontal_roads_under_construction(maze_image, nodes)]
	# print(vertical_roads_under_construction)
	
	for key , nodes in paths.items():
		for node in nodes.copy().keys():							#nodes.copy():
			road = set([key, node])
			if road in vertical_roads_under_construction or road in horizontal_roads_under_construction:
				paths[key].pop(node) 								# -= {node}
	##################################################

	return paths

def get_dist(n1,n2):
	# nodes = detect_all_nodes(maze_image)
	x1,y1 =nodes[n1]
	x2,y2 = nodes[n2]
	dist = np.sqrt((x1-x2)**2+(y1-y2)**2)
	return dist

def path_planning(paths, start, end):

	"""
	Purpose:
	---
	This function takes the graph(dict), start and end node for planning the shortest path

	** Note: You can use any path planning algorithm for this but need to produce the path in the form of 
	list given below **

	Input Arguments:
	---
	`graph` :	{ dictionary }
			dict of all connecting path
	`start` :	str
			name of start node
	`end` :		str
			name of end node
	Returns:
	---
	`backtrace_path` : [ list of nodes ]
			list of nodes, produced using path planning algorithm

		eg.: ['C6', 'C5', 'B5', 'B4', 'B3']
	
	Example call:
	---
	arena_parameters = detect_arena_parameters(maze_image)
	"""    

	backtrace_path=[]

	##############	ADD YOUR CODE HERE	##############
	f_costs = {start:get_dist(start, end)}
	open = [start]
	visited = []
	distances = {start:0}
	came_from = {}

	for node in open:
		
		if node == end:
			break
 			# retrace path
		if node in visited:
			continue

		for sucessor in paths[node]:
			f_costs[sucessor] = f_costs[node]+100 + get_dist(sucessor, end)
			if sucessor not in distances or distances[sucessor] > distances[node]+100:
				distances[sucessor] = distances[node]+ 100
				came_from[sucessor] = node
		f_costs.pop(node)
		f_costs = dict(sorted(f_costs.items(), key = lambda item: item[1]) )
		for key in f_costs.keys():
			if key not in open:
				open.append(key)
		visited.append(open[0])
	
	backtrace_path=[end]
	while end!=start:
		end=came_from[end]
		backtrace_path.append((end))

	backtrace_path.reverse()

	##################################################


	return backtrace_path

def paths_to_moves(paths, traffic_signal):

	"""
	Purpose:
	---
	This function takes the list of all nodes produces from the path planning algorithm
	and connecting both start and end nodes

	Input Arguments:
	---
	`paths` :	[ list of all nodes ]
			list of all nodes connecting both start and end nodes (SHORTEST PATH)
	`traffic_signal` : [ list of all traffic signals ]
			list of all traffic signals
	---
	`moves` : [ list of moves from start to end nodes ]
			list containing moves for the bot to move from start to end

			Eg. : ['UP', 'LEFT', 'UP', 'UP', 'RIGHT', 'DOWN']
	
	Example call:
	---
	moves = paths_to_moves(paths, traffic_signal)
	"""    
	
	list_moves=[]
	

	##############	ADD YOUR CODE HERE	##############
	Rt=0
	Dt=0

	for j in range(0,len(paths)-1):
		if (paths[j] in traffic_signal):
			list_moves.append('WAIT_5')

		if paths[j][0]>paths[j+1][0]:
			Dt=90
		elif paths[j][0]<paths[j+1][0]:
			Dt = 270
		elif paths[j][1]>paths[j+1][1]:
			Dt = 0
		elif paths[j][1]<paths[j+1][1]:
			Dt = 180


		# print('Rt: ', Rt)
		# print('Dt: ', Dt)

		O = Dt - Rt
		# print(O)
		Rt = Dt

		if O==90 or O==-270:
			list_moves.append('LEFT')
		elif O==-90 or O==270:
			list_moves.append('RIGHT')
		elif O==180 or O==-180:
			list_moves.append('REVERSE')
		else :
			list_moves.append('STRAIGHT')


	
	

	##################################################
	return list_moves



####################### ADD ARENA SETUP ######################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 4A for setting up the CoppeliaSim
Arena according to the configuration image in this section
"""
def get_node_locations():
    locations = {} 
    y = -0.8905

    for alpha in 'ABCDEF':
        x = 0.8905
        for i in range(1,7):
            locations[alpha+str(i)] = [y,x]
            x = round(x - 0.3565,5)
        y = round( y + 0.3565,5)

    return locations

def get_PN():
    PN = {}
    alpha = 'ABCDEF'
    for i in range(1,6):
        PN['Shop_'+str(i)] = alpha[i]+str(2)
    return PN
    
##############################################################

def pick_package(sim, pack_no, package):
	
	package_handle = sim.getObject('./Arena/'+ package[1]+'_'+package[2])
	alphabot = sim.getObject('./alphabot')
	sim.sim.setObjectParent(package_handle,alphabot, True)
	sim.setObjectPosition(package_handle ,alphabot ,[2.7497e-02,4.9961e-04+ pack_no*0.010,2.1111e-02 ])


def drop_packages(sim, package):
	package[1]+'_'+package[2]
	package_handle = sim.getObject('./Arena/'+ package[1]+'_'+package[2])
	arena = sim.getObject('./Arena')
	alphabot = sim.getObject('./alphabot')
	sim.setObjectParent(package_handle,arena, True)
	sim.setObjectPosition(package_handle ,alphabot ,[+1.7545e-02,-3.7388e-02,-3.2459e-02 ])

	

def place_packages(medicine_package_details, sim, all_models):
    """
	Purpose:
	---
	This function takes details (colour, shape and shop) of the packages present in 
    the arena (using "detect_arena_parameters" function from task_1a.py) and places
    them on the virtual arena. The packages should be inserted only into the 
    designated areas in each shop as mentioned in the Task document.

    Functions from Regular API References should be used to set the position of the 
    packages.

	Input Arguments:
	---
	`medicine_package_details` :	[ list ]
                                nested list containing details of the medicine packages present.
                                Each element of this list will contain 
                                - Shop number as Shop_n
                                - Color of the package as a string
                                - Shape of the package as a string
                                - Centroid co-ordinates of the package			

    `sim` : [ object ]
            ZeroMQ RemoteAPI object

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	Returns:

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	
	Example call:
	---
	all_models = place_packages(medicine_package_details, sim, all_models)
	"""
    models_directory = os.getcwd()
    packages_models_directory = os.path.join(models_directory, "package_models/")
    arena = sim.getObject('/Arena')    
####################### ADD YOUR CODE HERE #########################
    PN = get_PN()

    packages  = {'Shop_'+str(i): 0 for i in range(1,6)}
    locations = get_node_locations()

    for package_det in medicine_package_details:
        pack = packages_models_directory+package_det[1]+'_'+package_det[2]+'.ttm'
        # print(pack)
        package_handle = sim.loadModel(pack)
        sim.setObjectParent(package_handle,arena, True)
        sim.setObjectAlias(package_handle , package_det[1]+'_'+package_det[2])
        PN_loc = np.array( locations[PN[package_det[0]]] ) 
        # loc = PN_loc + [-0.155, 0.06] + [0, packages[package_det[0]]*0.08]
        loc = PN_loc + [-0.06, 0.155] + [-packages[package_det[0]]*0.08, 0]
        # print(loc)
        sim.setObjectPosition(package_handle ,arena ,[*loc,0.0255])
        packages[package_det[0]]+=1

        all_models.append(package_handle)


        
####################################################################
    return all_models

def place_traffic_signals(traffic_signals, sim, all_models):
    """
	Purpose:
	---
	This function takes position of the traffic signals present in 
    the arena (using "detect_arena_parameters" function from task_1a.py) and places
    them on the virtual arena. The signal should be inserted at a particular node.

    Functions from Regular API References should be used to set the position of the 
    signals.

	Input Arguments:
	---
	`traffic_signals` : [ list ]
			list containing nodes in which traffic signals are present

    `sim` : [ object ]
            ZeroMQ RemoteAPI object

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	Returns:

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	None
	
	Example call:
	---
	all_models = place_traffic_signals(traffic_signals, sim, all_models)
	"""
    models_directory = os.getcwd()
    traffic_sig_model = os.path.join(models_directory, "signals", "traffic_signal.ttm" )
    arena = sim.getObject('/Arena')   
####################### ADD YOUR CODE HERE #########################
    locations = get_node_locations()
    for signal in traffic_signals:
        signal_handle =  sim.loadModel(traffic_sig_model)
        sim.setObjectParent(signal_handle,arena, True)
        sim.setObjectAlias(signal_handle , 'Signal_'+signal)
        sim.setObjectPosition(signal_handle ,arena ,[*locations[signal], 0.1528])

        all_models.append(signal_handle)
####################################################################
    return all_models

def place_start_end_nodes(start_node, end_node, sim, all_models):
    """
	Purpose:
	---
	This function takes position of start and end nodes present in 
    the arena and places them on the virtual arena. 
    The models should be inserted at a particular node.

    Functions from Regular API References should be used to set the position of the 
    start and end nodes.

	Input Arguments:
	---
	`start_node` : [ string ]
    `end_node` : [ string ]
					

    `sim` : [ object ]
            ZeroMQ RemoteAPI object

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	Returns:

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	---
	None
	
	Example call:
	---
	all_models = place_start_end_nodes(start_node, end_node, sim, all_models)
	"""
    models_directory = os.getcwd()
    start_node_model = os.path.join(models_directory, "signals", "start_node.ttm" )
    end_node_model = os.path.join(models_directory, "signals", "end_node.ttm" )
    arena = sim.getObject('/Arena')   
####################### ADD YOUR CODE HERE #########################
    locations = get_node_locations()
    start_node_handle =sim.loadModel(start_node_model)
    sim.setObjectAlias(start_node_handle , 'Start_Node')
    sim.setObjectParent(start_node_handle,arena, True)
    end_node_handle =sim.loadModel(end_node_model)
    sim.setObjectAlias(end_node_handle , 'End_Node')
    sim.setObjectParent(end_node_handle,arena, True)

    sim.setObjectPosition(start_node_handle ,arena ,[*locations[start_node], 0.1528])
    sim.setObjectPosition(end_node_handle ,arena ,[*locations[end_node], 0.1528])
    all_models.append(start_node_handle)
    all_models.append(end_node_handle)


    
####################################################################
    return all_models

def place_horizontal_barricade(horizontal_roads_under_construction, sim, all_models):
    """
	Purpose:
	---
	This function takes the list of missing horizontal roads present in 
    the arena (using "detect_arena_parameters" function from task_1a.py) and places
    horizontal barricades on virtual arena. The barricade should be inserted 
    between two nodes as shown in Task document.

    Functions from Regular API References should be used to set the position of the 
    horizontal barricades.

	Input Arguments:
	---
	`horizontal_roads_under_construction` : [ list ]
			list containing missing horizontal links		

    `sim` : [ object ]
            ZeroMQ RemoteAPI object

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	Returns:

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	---
	None
	
	Example call:
	---
	all_models = place_horizontal_barricade(horizontal_roads_under_construction, sim, all_models)
	"""
    models_directory = os.getcwd()
    horiz_barricade_model = os.path.join(models_directory, "barricades", "horizontal_barricade.ttm" )
    arena = sim.getObject('/Arena')  
####################### ADD YOUR CODE HERE #########################
    locations = get_node_locations()
    for road in horizontal_roads_under_construction:
        barricade_handle =  sim.loadModel(horiz_barricade_model)
        sim.setObjectParent(barricade_handle,arena, True)

        
        node1 = np.array(locations[road[0]])
        node2 = np.array(locations[road[1]])
        # print(node1, node2)
        loc = (node1+node2)/2
        # print(loc)
        sim.setObjectAlias(barricade_handle , 'Horizontal_missing_road_'+road[0]+'_'+road[1])
        sim.setObjectPosition(barricade_handle ,arena ,[*loc, 0.0255])

        all_models.append(barricade_handle)
####################################################################
    return all_models


def place_vertical_barricade(vertical_roads_under_construction, sim, all_models):
    """
	Purpose:
	---
	This function takes the list of missing vertical roads present in 
    the arena (using "detect_arena_parameters" function from task_1a.py) and places
    vertical barricades on virtual arena. The barricade should be inserted 
    between two nodes as shown in Task document.

    Functions from Regular API References should be used to set the position of the 
    vertical barricades.

	Input Arguments:
	---
	`vertical_roads_under_construction` : [ list ]
			list containing missing vertical links		

    `sim` : [ object ]
            ZeroMQ RemoteAPI object

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	Returns:

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	---
	None
	
	Example call:
	---
	all_models = place_vertical_barricade(vertical_roads_under_construction, sim, all_models)
	"""
    models_directory = os.getcwd()
    vert_barricade_model = os.path.join(models_directory, "barricades", "vertical_barricade.ttm" )
    arena = sim.getObject('/Arena') 
####################### ADD YOUR CODE HERE #########################
    locations = get_node_locations()
    for road in vertical_roads_under_construction:
        barricade_handle =  sim.loadModel(vert_barricade_model)
        sim.setObjectParent(barricade_handle,arena, True)
        sim.setObjectAlias(barricade_handle , 'Vertical_missing_road_'+road[0]+'_'+road[1])
        node1 = np.array(locations[road[0]])
        node2 = np.array(locations[road[1]])
        # print(node1, node2)
        loc = (node1+node2)/2
        # print(loc)
        sim.setObjectPosition(barricade_handle ,arena ,[*loc, 0.0255])

        all_models.append(barricade_handle)
####################################################################
    return all_models


##############################################################
##############################################################



def perspective_transform(image):

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
    ArUco_details_dict, ArUco_corners  = task_1b.detect_ArUco_details(image)
    image_cp = numpy.copy(image)
    task_1b.mark_ArUco_image(image_cp, ArUco_details_dict, ArUco_corners)
    cv2.imshow('img_cp', image_cp)

    if len(ArUco_details_dict) == 5:
        pts = numpy.float32([numpy.array(ArUco_details_dict[1][0])+[10,10],numpy.array(ArUco_details_dict[2][0])+ [-10,10], numpy.array(ArUco_details_dict[3][0]) + [-10,-10], numpy.array(ArUco_details_dict[4][0]) + [10,-10]])
        # pts = numpy.float32([numpy.array(ArUco_details_dict[1][0]),numpy.array(ArUco_details_dict[2][0]), numpy.array(ArUco_details_dict[3][0]), numpy.array(ArUco_details_dict[4][0])])
        pts2 = numpy.float32([[512,512],[0,512], [0,0], [512,0]])
        mat = cv2.getPerspectiveTransform(pts, pts2)
        warped_image = cv2.warpPerspective(image, mat, [512,512])
        # cropped = image[ ArUco_details_dict[3][0][1]:ArUco_details_dict[1][0][1] , ArUco_details_dict[3][0][0]:ArUco_details_dict[1][0][0]]


        # cv2.imshow('cropped', cropped)
        cv2.imshow('img', warped_image)
    else: 
        print('5 not found')
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
        # print(a)

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


def set_values(scene_parameters):
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
######################################################################################

    return None
run = True
def start_emulation():
	global run
	run = True
	client = RemoteAPIClient()
	sim = client.getObject('sim')

#################################  ADD YOUR CODE HERE  ################################
    
    

	cam = cv2.VideoCapture('http://192.168.1.195:4747/video')
	la = 0
	while run:
		ret, frame = cam.read()
		frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

		
		
		try:
			Tframe = perspective_transform(frame)  
			scene_parameters = transform_values(Tframe)
			set_values(scene_parameters)

		except:
			pass

		# cv2.imshow('aruco', frame)  
		
		cv2.waitKey(1)

	# cv2.imshow('frame', frame)


#######################################################################################