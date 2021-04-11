#!/usr/bin/env python
#Author: Leandro Ponce
import rospy
import roslib
import os
import cv2
import numpy as np
import json
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String


import sys, random, math, pygame, time
from pygame.locals import *
from math import sqrt,cos,sin,atan2

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from path_planner import Ui_path_planner

# import  PyQt5 modules
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import *
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc

pub_waypoints = rospy.Publisher('/coord_waypoints_update_indicator', String,queue_size=10)

position = 0, 0
os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
#from aruco_msgs.msg import MarkerArray
obstaculos=[]
obstaculos_global=[]
aruco=[]
aruco_global=[]
ids=[]
ids_global=[]
angulos=[]
angulos_global=[]
waypoints_list=[]

vertices=[]
OBS=[]

def callback(data):
	
	os.system('clear')
	print("##########")
	print(data)
	with open('/home/proyinv/ponce_ws/src/cam_test/scripts/coord_obstaculos_aruco.json') as f:
		data_json = json.load(f)
	n=0
	obstaculos=[]
	for item in data_json['obstaculos']:
		index=item['index']
		coord=item['coord']
		obstaculos.append(coord)
		n=n+1
	obstaculos_global=obstaculos
	
	n=0
	aruco=[]
	for item in data_json['aruco']:
		index=item['index']
		coord=item['coord']
		aruco.append(coord)
		n=n+1
	aruco_global=aruco
	print("Aruco:")
	print(aruco)
	n=0
	ids=[]
	for item in data_json['ids']:
		index=item['index']
		id_aruco=item['id']
		ids.append(id_aruco)
		n=n+1
	ids_global=ids
	print("Aruco IDs:")
	print(ids)
	n=0
	angulos=[]
	for item in data_json['angulos']:
		index=item['index']
		aruco_angulo=item['angulo']
		angulos.append(aruco_angulo)
		n=n+1
	angulos_global=angulos
	print("Aruco angulos:")
	print(angulos)
	#cambiar de obstaculos[xy,xy,xy,xy a OBS para dibujar con pygame, requiere q cada vertice este delimitado con parentesis
	callback.OBS=[]
	vertices=[]
	for o in obstaculos_global: 
		vertices=[]
		for n in range(len(o)):
			
			if (n%2) == 0:
				#par
				vertices.append((o[n],o[n+1]))
			else:
				#impar
				pass
		callback.OBS.append(vertices)

	print("num de Obstaculos:")
	print(len(callback.OBS))
	print("Route nodes:")
	#waypoints_list=[]
	try:
		for each_node in path_planning.solution_nodes:
		
			print(each_node.x, each_node.y)
			#waypoints_list.append((each_node.x, each_node.y))
	except AttributeError:
		print('Aun no existe solucion ')
	
def get_line(x1, y1, x2, y2): #tomado de https://stackoverflow.com/questions/25837544/get-all-points-of-a-straight-line-in-python
	points = []
	issteep = abs(y2-y1) > abs(x2-x1)
	if issteep:
		x1, y1 = y1, x1
		x2, y2 = y2, x2
	rev = False
	if x1 > x2:
		x1, x2 = x2, x1
		y1, y2 = y2, y1
		rev = True
	deltax = x2 - x1
	deltay = abs(y2-y1)
	error = int(deltax / 2)
	y = y1
	ystep = None
	if y1 < y2:
		ystep = 1
	else:
		ystep = -1
	for x in range(x1, x2 + 1):
		if issteep:
			points.append((y, x))
		else:
			points.append((x, y))
		error -= deltay
		if error < 0:
			y += ystep
			error += deltax
	# Reverse the list if the coordinates were reversed
	if rev:
		points.reverse()
	return points
	
def my_checkIntersect(nodeA,nodeB,pygame,screen):#A,B son nodos
	A=(int(nodeA.x),int(nodeA.y))
	B=(int(nodeB.x),int(nodeB.y))
	puntos=get_line(A[0],A[1],B[0],B[1])
	#print(puntos)
	for pt in puntos:
		if (screen.get_at((pt))) == (0, 0, 255, 255): #R,G,B,Oppacity
			#print("there is obst")
			return False 
	return True

def obsDraw(pygame,screen,lista_obst_vert):
	blue=(0,0,255)
	
	for o in lista_obst_vert: 
		pygame.draw.polygon(screen, blue, o)
		#pygame.draw.rect(screen,blue,o)
	"""
	for o in OBS: 
		pygame.draw.rect(screen,blue,o)
	"""
def dist(p1,p2):
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
	if dist(p1,p2) < EPSILON:
		return p2
	else:
		theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
		return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def chooseParent(nn,newnode,nodes,screen):
	for p in nodes:
		#if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
		if my_checkIntersect(p,newnode,pygame,screen) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
			nn = p
	newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
	newnode.parent=nn
	return newnode,nn

def reWire(nodes,newnode,pygame,screen):
	white = 255, 240, 200
	black = 20, 20, 40
	for i in range(len(nodes)):
		p = nodes[i]
		#if checkIntersect(p,newnode,OBS) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
		if my_checkIntersect(p,newnode,pygame,screen) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
			pygame.draw.line(screen,white,[p.x,p.y],[p.parent.x,p.parent.y])  
			p.parent = newnode
			p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
			nodes[i]=p  
			pygame.draw.line(screen,black,[p.x,p.y],[newnode.x,newnode.y])                    
	return nodes

def drawSolutionPath(start,goal,nodes,pygame,screen):
	path_planning.solution_nodes=[]
	pink = 200, 20, 240
	nn = nodes[0]
	for p in nodes:
		if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
			nn = p
	while nn!=start:
		path_planning.solution_nodes.append(nn)
		pygame.draw.line(screen,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],2)  
		nn=nn.parent
	path_planning.solution_nodes.append(start)
	path_planning.solution_nodes.reverse()
class Cost:
	x = 0
	y = 0
	cost=0  
	parent=None
	def __init__(self,xcoord, ycoord):
		self.x = xcoord
		self.y = ycoord

class Node:
	x = 0
	y = 0
	cost=0  
	parent=None
	def __init__(self,xcoord, ycoord):
		xcoord_round=math.floor(xcoord)
		ycoord_round=math.floor(ycoord)
		if xcoord_round>XDIM:
			xcoord_round=XDIM
		if ycoord_round>YDIM:
			ycoord_round=YDIM
		self.x = xcoord_round
		self.y = ycoord_round


def path_planning():	
	############################# ROS SETTINGS FOR THE NODE ####################
	rospy.init_node('path_planning', anonymous=True)
	rospy.Subscriber("/coord_obstaculos_aruco", String, callback)#viene de mapa2d, mapa2d es el publisher
	
	
	class Worker4(qtc.QObject):
		'''
		Worker thread
		'''
		screen_signal4=qtc.pyqtSignal(pygame.Surface)
		@pyqtSlot()
		def run(self):
			'''
			Your code goes in this function
			'''
			screen4 = pygame.display.set_mode(WINSIZE)
			pygame.display.set_caption('RRTstar')
			white = 255, 255, 255
			black = 20, 20, 40
			screen4.fill(white)
			obsDraw(pygame,screen4,callback.OBS)#DIBUJAR OBSTACULOS
			
			path_planning.nodes = []
			   	
			path_planning.nodes.append(Node(300,600)) # Start in the center
			#path_planning.nodes.append(Node(XDIM/2,YDIM/2)) # Start in the center
			#nodes.append(Node(0.0,0.0)) # Start in the corner
			start=path_planning.nodes[0]
			goal=Node(330,100)
			for i in range(NUMNODES):
			   	
				#print("#### iteracion numero:",i)
				rand = Node(random.random()*XDIM,random.random()*YDIM)#Saca una coordenada random de X y Y
				nn = path_planning.nodes[0]
			       
				for p in path_planning.nodes:
					if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
						nn = p
				interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
				newnode = Node(interpolatedNode[0],interpolatedNode[1])

				if my_checkIntersect(nn,rand,pygame,screen4):
					[newnode,nn]=chooseParent(nn,newnode,path_planning.nodes,screen4);
					path_planning.nodes.append(newnode)
					pygame.draw.line(screen4,black,[nn.x,nn.y],[newnode.x,newnode.y])
					path_planning.nodes=reWire(path_planning.nodes,newnode,pygame,screen4)
					pygame.display.update()

			
			position = 0, 0
			os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
			drawSolutionPath(start,goal,path_planning.nodes,pygame,screen4)
			
			waypoints_list=[]
			for each_node in path_planning.solution_nodes:
				waypoints_list.append((each_node.x, each_node.y))
				
			waypoints_for_json=list(waypoints_list)
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json',"r+") as f:
				data_json = json.load(f)
			data_json['waypoints4']=[]
			for h in range(len(waypoints_for_json)):
				update={"coord":waypoints_for_json[h],"index":h}
				data_json['waypoints4'].append(update)
				
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json', 'w') as f:
				json.dump(data_json, f, indent=4)
			pub_waypoints.publish("json coord waypoints update")
			
			pygame.display.update()
			self.screen_signal4.emit(screen4)
	
	class Worker3(qtc.QObject):
		'''
		Worker thread
		'''
		screen_signal3=qtc.pyqtSignal(pygame.Surface)
		@pyqtSlot()
		def run(self):

			screen3 = pygame.display.set_mode(WINSIZE)
			pygame.display.set_caption('RRTstar')
			white = 255, 255, 255
			black = 20, 20, 40
			screen3.fill(white)
			obsDraw(pygame,screen3,callback.OBS)#DIBUJAR OBSTACULOS
			
			path_planning.nodes = []
			   	
			path_planning.nodes.append(Node(300,600)) # Start in the center
			#path_planning.nodes.append(Node(XDIM/2,YDIM/2)) # Start in the center
			#nodes.append(Node(0.0,0.0)) # Start in the corner
			start=path_planning.nodes[0]
			goal=Node(330,100)
			for i in range(NUMNODES):
			   	
				#print("#### iteracion numero:",i)
				rand = Node(random.random()*XDIM,random.random()*YDIM)#Saca una coordenada random de X y Y
				nn = path_planning.nodes[0]

				for p in path_planning.nodes:
					#print("nodes lista:", p.x,p.y)
					if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
						nn = p
				interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
			
				newnode = Node(interpolatedNode[0],interpolatedNode[1])

				if my_checkIntersect(nn,rand,pygame,screen3):
					[newnode,nn]=chooseParent(nn,newnode,path_planning.nodes,screen3);
					path_planning.nodes.append(newnode)
					pygame.draw.line(screen3,black,[nn.x,nn.y],[newnode.x,newnode.y])
					path_planning.nodes=reWire(path_planning.nodes,newnode,pygame,screen3)
					pygame.display.update()
			
			position = 0, 0
			os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
			drawSolutionPath(start,goal,path_planning.nodes,pygame,screen3)
			
			waypoints_list=[]
			for each_node in path_planning.solution_nodes:
				#print(each_node.x, each_node.y)
				waypoints_list.append((each_node.x, each_node.y))
				
			waypoints_for_json=list(waypoints_list)
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json',"r+") as f:
				data_json = json.load(f)
			data_json['waypoints3']=[]
			for h in range(len(waypoints_for_json)):
				update={"coord":waypoints_for_json[h],"index":h}
				data_json['waypoints3'].append(update)
				
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json', 'w') as f:
				json.dump(data_json, f, indent=4)
			pub_waypoints.publish("json coord waypoints update")
			
			pygame.display.update()
			self.screen_signal3.emit(screen3)


	class Worker2(qtc.QObject):
		'''
		Worker thread
		'''
		screen_signal2=qtc.pyqtSignal(pygame.Surface)
		@pyqtSlot()
		def run(self):

			screen2 = pygame.display.set_mode(WINSIZE)
			pygame.display.set_caption('RRTstar')
			white = 255, 255, 255
			black = 20, 20, 40
			screen2.fill(white)
			obsDraw(pygame,screen2,callback.OBS)#DIBUJAR OBSTACULOS

			path_planning.nodes = []
			   	
			path_planning.nodes.append(Node(300,600)) # Start in the center
			#path_planning.nodes.append(Node(XDIM/2,YDIM/2)) # Start in the center
			#nodes.append(Node(0.0,0.0)) # Start in the corner
			start=path_planning.nodes[0]
			goal=Node(330,100)
			for i in range(NUMNODES):

				rand = Node(random.random()*XDIM,random.random()*YDIM)#Saca una coordenada random de X y Y
				nn = path_planning.nodes[0]
			       
				for p in path_planning.nodes:
					if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
						nn = p
				interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
			
				newnode = Node(interpolatedNode[0],interpolatedNode[1])
				
				if my_checkIntersect(nn,rand,pygame,screen2):
					[newnode,nn]=chooseParent(nn,newnode,path_planning.nodes,screen2);
					path_planning.nodes.append(newnode)
					pygame.draw.line(screen2,black,[nn.x,nn.y],[newnode.x,newnode.y])
					path_planning.nodes=reWire(path_planning.nodes,newnode,pygame,screen2)
					pygame.display.update()
			
			position = 0, 0
			os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
			drawSolutionPath(start,goal,path_planning.nodes,pygame,screen2)
			
			waypoints_list=[]
			for each_node in path_planning.solution_nodes:
				#print(each_node.x, each_node.y)
				waypoints_list.append((each_node.x, each_node.y))
				
			waypoints_for_json=list(waypoints_list)
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json',"r+") as f:
				data_json = json.load(f)
			data_json['waypoints2']=[]
			for h in range(len(waypoints_for_json)):
				update={"coord":waypoints_for_json[h],"index":h}
				data_json['waypoints2'].append(update)
				
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json', 'w') as f:
				json.dump(data_json, f, indent=4)
			pub_waypoints.publish("json coord waypoints update")
			
			pygame.display.update()
			self.screen_signal2.emit(screen2)
	
	class Worker1(qtc.QObject):
		'''
		Worker thread
		'''
		screen_signal1=qtc.pyqtSignal(pygame.Surface)
		@pyqtSlot()
		def run(self):
			
			screen1 = pygame.display.set_mode(WINSIZE)
			pygame.display.set_caption('RRTstar')
			white = 255, 255, 255
			black = 20, 20, 40
			screen1.fill(white)
			obsDraw(pygame,screen1,callback.OBS)#DIBUJAR OBSTACULOS
			
			
			path_planning.nodes = []
			   	
			path_planning.nodes.append(Node(300,600)) # Start in the center
			#path_planning.nodes.append(Node(XDIM/2,YDIM/2)) # Start in the center
			#nodes.append(Node(0.0,0.0)) # Start in the corner
			start=path_planning.nodes[0]
			goal=Node(330,100)
			for i in range(NUMNODES):

				rand = Node(random.random()*XDIM,random.random()*YDIM)#Saca una coordenada random de X y Y
				nn = path_planning.nodes[0]
				
				for p in path_planning.nodes:
					if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
						nn = p
				interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
			
				newnode = Node(interpolatedNode[0],interpolatedNode[1])

				if my_checkIntersect(nn,rand,pygame,screen1):
					[newnode,nn]=chooseParent(nn,newnode,path_planning.nodes,screen1);
					path_planning.nodes.append(newnode)
					pygame.draw.line(screen1,black,[nn.x,nn.y],[newnode.x,newnode.y])
					path_planning.nodes=reWire(path_planning.nodes,newnode,pygame,screen1)
					pygame.display.update()
		
			
			position = 0, 0
			os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
			drawSolutionPath(start,goal,path_planning.nodes,pygame,screen1)
			
			waypoints_list=[]
			for each_node in path_planning.solution_nodes:
				#print(each_node.x, each_node.y)
				waypoints_list.append((each_node.x, each_node.y))
				
			waypoints_for_json=list(waypoints_list)
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json',"r+") as f:
				data_json = json.load(f)
			data_json['waypoints1']=[]
			for h in range(len(waypoints_for_json)):
				update={"coord":waypoints_for_json[h],"index":h}
				data_json['waypoints1'].append(update)
				
			with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json', 'w') as f:
				json.dump(data_json, f, indent=4)
			pub_waypoints.publish("json coord waypoints update")
			
			pygame.display.update()
			self.screen_signal1.emit(screen1)
	
	############################################### RRT STAR INITIALIZING AND FUNCTIONS #########################
	
				
	global XDIM, YDIM, WINSIZE, EPSILON, NUMNODES, RADIUS	
	global screen1,screen2,screen3,screen4
	#constants
	XDIM = 1280
	YDIM = 720
	WINSIZE = [XDIM, YDIM]
	EPSILON = 100.0
	NUMNODES = 300
	RADIUS=50
	#OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50)]

		
	pygame.init()
	screen4 = pygame.display.set_mode(WINSIZE)
	screen3 = pygame.display.set_mode(WINSIZE)
	screen2 = pygame.display.set_mode(WINSIZE)
	screen1 = pygame.display.set_mode(WINSIZE)
	
	###################################### APP WINDOW PYQT5 #####################################################
	
	class path_planner(qtw.QMainWindow,Ui_path_planner): #EL PRIMER ARGUMENTO DEBE COINCIDIR CON LO ESCOGIDO EN DESIGNER
		boton_click4 = qtc.pyqtSignal(str)
		boton_click3 = qtc.pyqtSignal(str)
		boton_click2 = qtc.pyqtSignal(str)
		boton_click1 = qtc.pyqtSignal(str)
		def __init__(self,screen4,screen3,screen2,screen1):
			#self.threadpool = QThreadPool()
			#print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
			super(path_planner,self).__init__()
			self.setupUi(self)
			# WORKER 4 and init screen 4
			w4=screen4.get_width()
			h4=screen4.get_height()
			self.data4=screen4.get_buffer().raw
			self.image4=QImage(self.data4,w4,h4,QImage.Format_RGB32)
			self.start_button_4.clicked.connect(self.show_pygame4)
			self.worker4 = Worker4()
			self.worker_thread4 = qtc.QThread()
			self.worker4.screen_signal4.connect(self.show_window4)
			self.boton_click4.connect(self.worker4.run)
			self.worker4.moveToThread(self.worker_thread4)
			self.worker_thread4.start()
			# WORKER 3 and init screen 3
			w3=screen3.get_width()
			h3=screen3.get_height()
			self.data3=screen3.get_buffer().raw
			self.image3=QImage(self.data3,w3,h3,QImage.Format_RGB32)
			self.start_button_3.clicked.connect(self.show_pygame3)
			self.worker3 = Worker3()
			self.worker_thread3 = qtc.QThread()
			self.worker3.screen_signal3.connect(self.show_window3)
			self.boton_click3.connect(self.worker3.run)
			self.worker3.moveToThread(self.worker_thread3)
			self.worker_thread3.start()
			# WORKER 2 and init screen 2
			w2=screen2.get_width()
			h2=screen2.get_height()
			self.data2=screen2.get_buffer().raw
			self.image2=QImage(self.data2,w2,h2,QImage.Format_RGB32)
			self.start_button_2.clicked.connect(self.show_pygame2)
			self.worker2 = Worker2()
			self.worker_thread2 = qtc.QThread()
			self.worker2.screen_signal2.connect(self.show_window2)
			self.boton_click2.connect(self.worker2.run)
			self.worker2.moveToThread(self.worker_thread2)
			self.worker_thread2.start()
			# WORKER 1 and init screen 1
			w1=screen1.get_width()
			h1=screen1.get_height()
			self.data1=screen1.get_buffer().raw
			self.image1=QImage(self.data1,w1,h1,QImage.Format_RGB32)
			self.start_button_1.clicked.connect(self.show_pygame1)
			self.worker1 = Worker1()
			self.worker_thread1 = qtc.QThread()
			self.worker1.screen_signal1.connect(self.show_window1)
			self.boton_click1.connect(self.worker1.run)
			self.worker1.moveToThread(self.worker_thread1)
			self.worker_thread1.start()
			#pygame.quit()#PARA ELIMINAR LA VENTANA PYGAME Q SE CREA
			
			
		def paintEvent(self,e):
			qp=QPainter()
			qp.begin(self)
			self.image4.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image4.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result4 = self.image4.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(500,400,self.result4)
			
			self.image3.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image3.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result3 = self.image3.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(10,400,self.result3)
			
			self.image2.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image2.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result2 = self.image2.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(500,70,self.result2)
			
			self.image1.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image1.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result1 = self.image1.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(10,70,self.result1)
			
			qp.end()
		########################################
		def show_pygame4(self):
			self.boton_click4.emit("clicked")
		def show_pygame3(self):
			self.boton_click3.emit("clicked")
		def show_pygame2(self):
			self.boton_click2.emit("clicked")
		def show_pygame1(self):
			self.boton_click1.emit("clicked")
			
		def show_window4(self,screen4):
			w4=screen4.get_width()
			h4=screen4.get_height()
			self.data4=screen4.get_buffer().raw
			self.image4=QImage(self.data4,w4,h4,QImage.Format_RGB32)
		def show_window3(self,screen3):
			w3=screen3.get_width()
			h3=screen3.get_height()
			self.data3=screen3.get_buffer().raw
			self.image3=QImage(self.data3,w3,h3,QImage.Format_RGB32)
		def show_window2(self,screen2):
			w2=screen2.get_width()
			h2=screen2.get_height()
			self.data2=screen2.get_buffer().raw
			self.image2=QImage(self.data2,w2,h2,QImage.Format_RGB32)
		def show_window1(self,screen1):
			w1=screen1.get_width()
			h1=screen1.get_height()
			self.data1=screen1.get_buffer().raw
			self.image1=QImage(self.data1,w1,h1,QImage.Format_RGB32)
			
			
	app = qtw.QApplication([])
	appWindow = path_planner(screen4,screen3,screen2,screen1)
	appWindow.show()
	sys.exit(app.exec_())

	rospy.spin()
	#cv2.destroyAllWindows()


if __name__ == '__main__':
	#ros
	path_planning()
	#pyqt5
	#app.exec_()
	


