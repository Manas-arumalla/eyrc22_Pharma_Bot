'''
*****************************************************************************************
*
*        		===============================================
*           		Pharma Bot (PB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 5 of Pharma Bot (PB) Theme (eYRC 2022-23).
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
# Filename:			task_5.py
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
import random
##############################################################

## Import PB_theme_functions code
try:
	pb_theme = __import__('PB_theme_functions')

except ImportError:
	print('\n[ERROR] PB_theme_functions.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure PB_theme_functions.py is present in this current directory.\n')
	sys.exit()
	
except Exception as e:
	print('Your PB_theme_functions.py throwed an Exception, kindly debug your code!\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

def nearest_nodes(start,end_nodes, arena_parameters):
	paths_lens=[]
	paths=[]
	for i in end_nodes:
		back_path = pb_theme.path_planning(arena_parameters["paths"],start, i)
		# print(back_path)
		moves = pb_theme.paths_to_moves(back_path, arena_parameters["traffic_signals"])
		# print(moves)
		path_len = len(moves)
		paths_lens.append(path_len)
		paths.append(moves)
		# print(paths)
		
		
	index = paths_lens.index(min(paths_lens))
	# print(list(end_nodes)[index])
	return list(end_nodes)[index] , paths[index]

	

def task_5_implementation(sim, arena_parameters):
	"""
	Purpose:
	---
	This function contains the implementation logic for task 5 

	Input Arguments:
	---
    `sim` : [ object ]
            ZeroMQ RemoteAPI object

	You are free to define additional input arguments for this function.

	Returns:
	---
	You are free to define output parameters for this function.
	
	Example call:
	---
	task_5_implementation(sim)
	"""

	##################	ADD YOUR CODE HERE	##################
	medicine_packages=arena_parameters['medicine_packages']

	nodes_shop = {'Shop_1':'B2','Shop_2':'C2','Shop_3':'D2','Shop_4':'E2','Shop_5':'F2'}
	shop_nodes = {'B2':'Shop_1','C2':'Shop_2','D2':'Shop_3','E2':'Shop_4','F2':'Shop_5'}



	# print(PN)
	
	pack={"Shop_1":[],"Shop_2":[],"Shop_3":[],"Shop_4":[],"Shop_5":[]}

	for i in arena_parameters["medicine_packages"]:
		pack[i[0]].append(i[1])
	print(pack)

	start=arena_parameters['start_node']
	print()

	NOP= len(arena_parameters["medicine_packages"])
	counter=NOP//3
	if NOP%3>0:
		counter+=1

	pb_theme.send_message_via_socket(connection_2, str(counter))

	for count in range(counter):	
		deli=[]
		picked_up=[]
		for i in range(3):
			if len(medicine_packages)==0:
				pb_theme.send_message_via_socket(connection_2, 'break')
				break

			PN = { nodes_shop[detail[0]] for detail in  medicine_packages}
			end_nodes=PN
			node , moves =nearest_nodes(start, end_nodes,  arena_parameters)
			print(moves)
			# send path to RPi
			path = ''
			for move in moves:
				path = path + move+','
			pb_theme.send_message_via_socket(connection_2,path )
			message = pb_theme.receive_message_via_socket(connection_2)
			print(message)
			colour=pack[shop_nodes[node]].pop(0)
	
		
			for i,package in enumerate(medicine_packages):
				if package[:2]==[shop_nodes[node],colour]:
					picked=medicine_packages.pop(i)
					pb_theme.pick_package(sim, i, picked)
					picked_up.append(picked)
			
			pb_theme.send_message_via_socket(connection_2, picked[1])
			
			print("PICKED UP: ",picked[1],",",picked[2])
			pb_theme.receive_message_via_socket(connection_2)
			print("#############")
			print(picked)
			print("###################")
			start=node
			print(node)
			alphabot = sim.getObject('/alphabot')
			pb_theme.activateQr(sim,node)
			#setting robot position
			if node=="F2":
				sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
				sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
				sim.setObjectPosition(alphabot,sim.handle_world ,[-8.8900e-01,-4.3361e-01,+3.2500e-02])
			elif node=="E2": 
				sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
				sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
				sim.setObjectPosition(alphabot,sim.handle_world ,[-5.3673e-01,-4.3434e-01,+3.2500e-02])
			elif node=="D2": 
				sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
				sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
				sim.setObjectPosition(alphabot,sim.handle_world ,[-1.7773e-01,-4.3134e-01,+3.2500e-02])
			elif node=="C2": 
				sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
				sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
				sim.setObjectPosition(alphabot,sim.handle_world ,[+1.7227e-01,-4.3361e-01,+3.2459e-02])
			elif node=="B2": 
				sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
				sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
				sim.setObjectPosition(alphabot,sim.handle_world ,[+5.3173e-01,-4.3361e-01,+3.2459e-02])
			time.sleep(1)

				
			qr_message = pb_theme.read_qr_code(sim)																																				
			pb_theme.deactivateQr(sim, node)
			print(qr_message)
			# for i in qr_message.keys():
			# 	for j in picked[1:2]:
			# 		if picked[1]==i.split("_")[0] and picked[2]==i.split("_")[1]:
			# 			deli.append(qr_message[i])
			deli.append(qr_message[picked[1]+'_'+picked[2]])
		print()	
		print(deli)
		print()
		delicopy = deli.copy()
		pb_theme.send_message_via_socket(connection_2, str(len(deli)))
		for i in range(len(deli)):
			# sta.append(node)
			# del_nodes,path=nearest_nodes(sta[0],deli,arena_parameters)
			# print("...............................")
			# print(del_nodes)
			# sta.pop(0)
			# sta.append(del_nodes)
			# k=deli.index(del_nodes)
			# deli.pop(k)
			# print(deli)

			node , path=nearest_nodes(start, deli,  arena_parameters)
			print("...............................")
			print(node)
			index=delicopy.index(node)
			
			pb_theme.send_message_via_socket(connection_2, str(index))
			pb_theme.receive_message_via_socket(connection_2)
			print("DELIVERED AT: ",picked_up[index][1]+", "+picked_up[index][2]+', ', node)
			deli.remove(node)
			print(deli)
			print(picked_up[index])
			pb_theme.drop_packages(sim, picked_up[index])
			start = node
			time.sleep(1)



		# for i in shop_nodes.keys():
		# 	if i==:
		# 		pack[shop_nodes[i]].pop[0]
		# print("delivered")
		
		


		

		




	

	##########################################################

if __name__ == "__main__":
	
	host = ''
	port = 4948


	## Set up new socket server
	try:
		server = pb_theme.setup_server(host, port)
		print("Socket Server successfully created")

		# print(type(server))

	except socket.error as error:
		print("Error in setting up server")
		print(error)
		sys.exit()


	## Set up new connection with a socket client (PB_task3d_socket.exe)
	try:
		print("\nPlease run PB_socket.exe program to connect to PB_socket client")
		connection_1, address_1 = pb_theme.setup_connection(server)
		print("Connected to: " + address_1[0] + ":" + str(address_1[1]))

	except KeyboardInterrupt:
		sys.exit()


	## Set up new connection with a socket client (socket_client_rgb.py)
	try:
		print("\nPlease connect to Raspberry pi client")
		connection_2, address_2 = pb_theme.setup_connection(server)
		print("Connected to: " + address_2[0] + ":" + str(address_2[1]))

	except KeyboardInterrupt:
		sys.exit()

	## Send setup message to PB_socket
	pb_theme.send_message_via_socket(connection_1, "SETUP")

	message = pb_theme.receive_message_via_socket(connection_1)
	## Loop infinitely until SETUP_DONE message is received
	while True:
		if message == "SETUP_DONE":
			break
		else:
			print("Cannot proceed further until SETUP command is received")
			message = pb_theme.receive_message_via_socket(connection_1)

	## Send setup message to PB_socket
	pb_theme.send_message_via_socket(connection_1, "GET_CONFIG")

	message = pb_theme.receive_message_via_socket(connection_1)
	## Loop infinitely until configuration message is received
	while True:
		if message == "CONFIG_1" or message == "CONFIG_2" or message == "CONFIG_3" or message == "CONFIG_4" or message == "CONFIG_5":
			break
		else:
			print("Cannot proceed further until Configuration is received")
			message = pb_theme.receive_message_via_socket(connection_1)

	try:
		
		# obtain required arena parameters
		image_filename = os.path.join(os.getcwd(), message.lower(), "config_image.png")
		config_img = cv2.imread(image_filename)
		detected_arena_parameters = pb_theme.detect_arena_parameters(config_img)			
		medicine_package_details = detected_arena_parameters["medicine_packages"]
		traffic_signals = detected_arena_parameters['traffic_signals']
		start_node = detected_arena_parameters['start_node']
		end_node = detected_arena_parameters['end_node']
		horizontal_roads_under_construction = detected_arena_parameters['horizontal_roads_under_construction']
		vertical_roads_under_construction = detected_arena_parameters['vertical_roads_under_construction']

		# print("Medicine Packages: ", medicine_package_details)
		# print("Traffic Signals: ", traffic_signals)
		# print("Start Node: ", start_node)
		# print("End Node: ", end_node)
		# print("Horizontal Roads under Construction: ", horizontal_roads_under_construction)
		# print("Vertical Roads under Construction: ", vertical_roads_under_construction)
		# print("\n\n")

	except Exception as e:
		print('Your task_1a.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()

	try:

		## Connect to CoppeliaSim arena
		coppelia_client = RemoteAPIClient()
		sim = coppelia_client.getObject('sim')

		## Define all models
		all_models = []

		## Setting up coppeliasim scene
		print("[1] Setting up the scene in CoppeliaSim")
		all_models = pb_theme.place_packages(medicine_package_details, sim, all_models)
		all_models = pb_theme.place_traffic_signals(traffic_signals, sim, all_models)
		all_models = pb_theme.place_horizontal_barricade(horizontal_roads_under_construction, sim, all_models)
		all_models = pb_theme.place_vertical_barricade(vertical_roads_under_construction, sim, all_models)
		all_models = pb_theme.place_start_end_nodes(start_node, end_node, sim, all_models)
		print("[2] Completed setting up the scene in CoppeliaSim")
		print("[3] Checking arena configuration in CoppeliaSim")

	except Exception as e:
		print('Your task_4a.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()


	pb_theme.send_message_via_socket(connection_1, "CHECK_ARENA")

	## Check if arena setup is ok or not
	message = pb_theme.receive_message_via_socket(connection_1)
	while True:
		# message = pb_theme.receive_message_via_socket(connection_1)

		if message == "ARENA_SETUP_OK":
			print("[4] Arena was properly setup in CoppeliaSim")
			break
		elif message == "ARENA_SETUP_NOT_OK":
			print("[4] Arena was not properly setup in CoppeliaSim")
			connection_1.close()
			# connection_2.close()
			server.close()
			sys.exit()
		else:
			pass

	## Send Start Simulation Command to PB_Socket
	pb_theme.send_message_via_socket(connection_1, "SIMULATION_START")
	
	## Check if simulation started correctly
	message = pb_theme.receive_message_via_socket(connection_1)
	while True:
		# message = pb_theme.receive_message_via_socket(connection_1)

		if message == "SIMULATION_STARTED_CORRECTLY":
			print("[5] Simulation was started in CoppeliaSim")
			break

		if message == "SIMULATION_NOT_STARTED_CORRECTLY":
			print("[5] Simulation was not started in CoppeliaSim")
			sys.exit()

	# send_message_via_socket(connection_2, "START")

	task_5_implementation(sim,detected_arena_parameters)


	## Send Stop Simulation Command to PB_Socket
	pb_theme.send_message_via_socket(connection_1, "SIMULATION_STOP")

	## Check if simulation started correctly
	message = pb_theme.receive_message_via_socket(connection_1)
	while True:
		# message = pb_theme.receive_message_via_socket(connection_1)

		if message == "SIMULATION_STOPPED_CORRECTLY":
			print("[6] Simulation was stopped in CoppeliaSim")
			break

		if message == "SIMULATION_NOT_STOPPED_CORRECTLY":
			print("[6] Simulation was not stopped in CoppeliaSim")
			sys.exit()