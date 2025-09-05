import os, sys
import cv2
from zmqRemoteApi import RemoteAPIClient
import numpy as np
import zmq
import PB_theme_functions as  pb_theme
import json
import random





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



image_filename = os.path.join(os.getcwd(), 'config_5', "config_image.png")
config_img = cv2.imread(image_filename)
arena_parameters = pb_theme.detect_arena_parameters(config_img)

medicine_packages=arena_parameters['medicine_packages']

nodes_shop = {'Shop_1':'B2','Shop_2':'C2','Shop_3':'D2','Shop_4':'E2','Shop_5':'F2'}
shop_nodes = {'B2':'Shop_1','C2':'Shop_2','D2':'Shop_3','E2':'Shop_4','F2':'Shop_5'}



# print(PN)
deli=[]
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

coppelia_client = RemoteAPIClient()
sim = coppelia_client.getObject('sim')

for count in range(counter):	
	for i in range(3):
		if len(medicine_packages)==0:
			break

		PN = { nodes_shop[detail[0]] for detail in  medicine_packages}
		end_nodes=PN
		node , path=nearest_nodes(start, end_nodes,  arena_parameters)
		# print("33333333333333333333")
		# print(node, path)

		# robot reaches PN
		#
		colour=pack[shop_nodes[node]].pop(0)
		for i,package in enumerate(medicine_packages):
			if package[:2]==[shop_nodes[node],colour]:
				picked=medicine_packages.pop(i)
				print()
				print(picked)
				print()
		print(pack)
		print("PICKED UP: ",picked[1],",",picked[2])
		start=node
		alphabot = sim.getObject('/alphabot')
		pb_theme.activateQr(sim,end_nodes)
		#setting robot position
		if node=="F5":
			sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
			sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
			sim.setObjectPosition(alphabot,sim.handle_world ,[-8.8900e-01,-4.3361e-01,+3.2500e-02])
		elif node=="E5": 
			sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
			sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
			sim.setObjectPosition(alphabot,sim.handle_world ,[-5.3673e-01,-4.3434e-01,+3.2500e-02])
		elif node=="D5": 
			sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
			sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
			sim.setObjectPosition(alphabot,sim.handle_world ,[-1.7773e-01,-4.3134e-01,+3.2500e-02])
		elif node=="C5": 
			sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
			sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
			sim.setObjectPosition(alphabot,sim.handle_world ,[+1.7227e-01,-4.3361e-01,+3.2459e-02])
		elif node=="B5": 
			sim.setObjectOrientation(alphabot,sim.getObject('/Arena'),[0,-89.5,0])
			sim.setObjectOrientation(alphabot,alphabot,[-89.5,0,0])
			sim.setObjectPosition(alphabot,sim.handle_world ,[+5.3673e-01,-4.3361e-01,+3.2459e-02])
			
		qr_message = pb_theme.read_qr_code(sim)																																				
		pb_theme.deactivateQr(sim, end_nodes)
		print(qr_message)
		for i in qr_message.keys():
			for j in picked[1:2]:
				if picked[1]==i.split("_")[0] and picked[2]==i.split("_")[1]:
					deli.append(qr_message[i])
		
	# for i in range(3):
	# 	node,path=nearest_nodes(start, , arena_parameters)
	# for i in shop_nodes.keys():
	# 	if i==node:
	# 		pack[shop_nodes[i]].pop[0]
	print("delivered")

nearest_del=nearest_node_path(start,deli,arena_parameters)
print(nearest_del)