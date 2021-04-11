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
from PyQt5.QtWidgets import QLabel 

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

def callback_imagen(data):
	br = CvBridge()
	np_arr = np.fromstring(data.data, np.uint8)
	callback_imagen.image_np = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR)


def callback(data):
	
	os.system('clear')
	print("########## PATH PLANNER RUNNING ##########")
	#print(data)
	with open('/home/proyinv/ponce_ws/src/cam_test/scripts/coord_obstaculos_aruco.json') as f: #viene de mapa2d
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
	#print("Aruco:")
	#print(aruco)
	n=0
	ids=[]
	for item in data_json['ids']:
		index=item['index']
		id_aruco=item['id']
		ids.append(id_aruco)
		n=n+1
	ids_global=ids
	#print("Aruco IDs:")
	#print(ids)
	n=0
	angulos=[]
	for item in data_json['angulos']:
		index=item['index']
		aruco_angulo=item['angulo']
		angulos.append(aruco_angulo)
		n=n+1
	angulos_global=angulos
	#print("Aruco angulos:")
	#print(angulos)
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
'''
	print("num de Obstaculos:")
	print(len(callback.OBS))
	print("Route nodes, last solution:")
	#waypoints_list=[]
	try:
		for each_node in path_planning_all.solution_nodes:
		
			print(each_node.x, each_node.y)
			#waypoints_list.append((each_node.x, each_node.y))
	except AttributeError:
		print('Aun no existe solucion ')
	
'''	
	
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
	path_planning_all.solution_nodes=[]
	pink = 200, 20, 240
	nn = nodes[0]
	for p in nodes:
		if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
			nn = p
	while nn!=start:
		path_planning_all.solution_nodes.append(nn)
		pygame.draw.line(screen,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],2)  
		nn=nn.parent
	path_planning_all.solution_nodes.append(start)
	path_planning_all.solution_nodes.reverse()
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


def path_planning_all():

	############################# ROS SETTINGS FOR THE NODE ####################
	rospy.init_node('path_planning_all', anonymous=True)
	rospy.Subscriber("/coord_obstaculos_aruco", String, callback)#viene de mapa2d, mapa2d es el publisher
	rospy.Subscriber('/imagen_procesada', CompressedImage, callback_imagen)
	
	class Worker(qtc.QObject):
		'''
		Worker thread
		'''
		screen_signal=qtc.pyqtSignal(pygame.Surface)

		@pyqtSlot()
		def run(self):
			'''
			Your code goes in this function
			'''
			screen = pygame.display.set_mode(WINSIZE)

			pygame.display.set_caption('RRTstar')
			white = 255, 255, 255
			black = 20, 20, 40
			screen.fill(white)
			obsDraw(pygame,screen,callback.OBS)#DIBUJAR OBSTACULOS
			path_planning_all.nodes = []
			
			start_x=path_planner.start_x
			start_y=path_planner.start_y
			path_planning_all.nodes.append(Node(start_x,start_y)) # Start in the center
			#path_planning_all.nodes.append(Node(XDIM/2,YDIM/2)) # Start in the center
			#nodes.append(Node(0.0,0.0)) # Start in the corner
			start=path_planning_all.nodes[0]
			
			finish_x=path_planner.finish_x
			finish_y=path_planner.finish_y
			
			goal=Node(finish_x,finish_y)
			for i in range(NUMNODES):
			   	
				#print("#### iteracion numero:",i)
				rand = Node(random.random()*XDIM,random.random()*YDIM)#Saca una coordenada random de X y Y
				nn = path_planning_all.nodes[0]
			       
				for p in path_planning_all.nodes:
					if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
						nn = p
				interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
				newnode = Node(interpolatedNode[0],interpolatedNode[1])

				if my_checkIntersect(nn,rand,pygame,screen):
					[newnode,nn]=chooseParent(nn,newnode,path_planning_all.nodes,screen);
					path_planning_all.nodes.append(newnode)
					pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
					path_planning_all.nodes=reWire(path_planning_all.nodes,newnode,pygame,screen)
					pygame.display.update()

			
			position = 0, 0
			os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
			drawSolutionPath(start,goal,path_planning_all.nodes,pygame,screen)
			
			pygame.display.update()
			self.screen_signal.emit(screen)

	
	
	############################################### RRT STAR INITIALIZING AND FUNCTIONS #########################
	
				
	global XDIM, YDIM, WINSIZE, EPSILON, NUMNODES, RADIUS	
	global screen
	#constants
	XDIM = 1280
	YDIM = 720
	WINSIZE = [XDIM, YDIM]
	EPSILON = 100.0
	NUMNODES = 1000
	RADIUS=100
	#OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50)]

		
	pygame.init()
	screen = pygame.display.set_mode(WINSIZE)
	
	###################################### APP WINDOW PYQT5 #####################################################
	
	class path_planner(qtw.QMainWindow,Ui_path_planner): #EL PRIMER ARGUMENTO DEBE COINCIDIR CON LO ESCOGIDO EN DESIGNER

		boton_click = qtc.pyqtSignal(str)
		def __init__(self,screen):
			global mac_num
			#self.threadpool = QThreadPool()
			#print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
			super(path_planner,self).__init__()
			self.setupUi(self)
			self.timer = QTimer()
			self.timer.timeout.connect(self.verCam)
			self.start_button.clicked.connect(self.controlTimer)
			
			path_planner.mac_num=self.code_edit.text()
			path_planner.start_x=640
			path_planner.start_y=360
			path_planner.finish_x=640
			path_planner.finish_y=720
			# WORKER 1 and init screen 1
			w=screen.get_width()
			h=screen.get_height()
			self.data=screen.get_buffer().raw
			self.image4=QImage(self.data,w,h,QImage.Format_RGB32)
			self.image3=QImage(self.data,w,h,QImage.Format_RGB32)
			self.image2=QImage(self.data,w,h,QImage.Format_RGB32)
			self.image1=QImage(self.data,w,h,QImage.Format_RGB32)
			#if mac_num != '':
			self.routes_button.clicked.connect(self.show_pygame)
			self.worker = Worker()
			self.worker_thread = qtc.QThread()
			self.worker.screen_signal.connect(self.show_window)
			self.boton_click.connect(self.worker.run)
			self.worker.moveToThread(self.worker_thread)
			self.worker_thread.start()
			#pygame.quit()#PARA ELIMINAR LA VENTANA PYGAME Q SE CREA
			
			
		def paintEvent(self,e):
			qp=QPainter()
			qp.begin(self)
			
			self.image4.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image3.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result4 = self.image4.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(1320,755,self.result4)
			
			self.image3.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image3.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result3 = self.image3.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(1320,510,self.result3)
			
			self.image2.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image2.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result2 = self.image2.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(1320,265,self.result2)
			
			self.image1.scaledToWidth (320,qtc.Qt.SmoothTransformation)
			self.image1.scaledToHeight(240,qtc.Qt.SmoothTransformation)
			self.result1 = self.image1.scaled(320, 240, qtc.Qt.KeepAspectRatioByExpanding, qtc.Qt.SmoothTransformation)
			qp.drawImage(1320,20,self.result1)
			
			qp.end()
		########################################
		def show_pygame(self):
			if self.edit_x_start.text()=='':
				path_planner.start_x=640
				self.edit_x_start.setText(str(640))
			if self.edit_y_start.text()=='':
				path_planner.start_y=360
				self.edit_y_start.setText(str(360))
			if self.edit_x_finish.text()=='':
				path_planner.finish_x=640
				self.edit_x_finish.setText(str(640))
			if self.edit_y_finish.text()=='':
				path_planner.finish_y=720
				self.edit_y_finish.setText(str(720))
				
			path_planner.start_x=int(self.edit_x_start.text())
			path_planner.start_y=int(self.edit_y_start.text())
			path_planner.finish_x=int(self.edit_x_finish.text())
			path_planner.finish_y=int(self.edit_y_finish.text())
			self.boton_click.emit("clicked")
			
		def show_window(self,screen):
			if self.code_edit.text() == '0013A2004191C5FB' or self.code_edit.text() == '4':
				w=screen.get_width()
				h=screen.get_height()
				self.data=screen.get_buffer().raw
				self.image4=QImage(self.data,w,h,QImage.Format_RGB32)
				
				waypoints_list=[]
				for each_node in path_planning_all.solution_nodes:
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
				
			if self.code_edit.text() == '0013A2004191C5ED' or self.code_edit.text() == '3':
				w=screen.get_width()
				h=screen.get_height()
				self.data=screen.get_buffer().raw
				self.image3=QImage(self.data,w,h,QImage.Format_RGB32)
				waypoints_list=[]
				for each_node in path_planning_all.solution_nodes:
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
				
			if self.code_edit.text() == '0013A2004191C5E1' or self.code_edit.text() == '2':
				w=screen.get_width()
				h=screen.get_height()
				self.data=screen.get_buffer().raw
				self.image2=QImage(self.data,w,h,QImage.Format_RGB32)
				
				waypoints_list=[]
				for each_node in path_planning_all.solution_nodes:
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
				
			if self.code_edit.text() == '0013A2004191782D' or self.code_edit.text() == '1':
				w=screen.get_width()
				h=screen.get_height()
				self.data=screen.get_buffer().raw
				self.image1=QImage(self.data,w,h,QImage.Format_RGB32)
				waypoints_list=[]
				for each_node in path_planning_all.solution_nodes:
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
				
			if self.code_edit.text() == '':
				qtw.QMessageBox.information(self,'Codigo vacio','La solucion se asignara al ID=1')
				w=screen.get_width()
				h=screen.get_height()
				self.data=screen.get_buffer().raw
				self.image1=QImage(self.data,w,h,QImage.Format_RGB32)
				waypoints_list=[]
				for each_node in path_planning_all.solution_nodes:
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

		def verCam(self):
			
			image = callback_imagen.image_np
			image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
			cv2.circle(image,(path_planner.start_x,path_planner.start_y), 10, (0,0,255), -1)
			cv2.circle(image,(path_planner.finish_x,path_planner.finish_y), 10, (255,0,0), -1)
			height, width, channel = image.shape # Saca la informacion de la imagen
			step = channel * width
			# create QImage from image
			self.label_video.mousePressEvent = self.getPos
			
			self.qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)#Formato para mostrar en pyqt5
			self.label_video.setPixmap(QPixmap.fromImage(self.qImg))# Se muestra en label_video con Qpixmap
			
			#self.label_video.setPixmap(QPixmap.fromImage(qImg)).mousePressEvent = self.getPos

		def getPos(self , event):

			if self.checkBox_start.isChecked():
				x = event.pos().x()
				y = event.pos().y() 
				self.edit_x_start.setText(str(x))
				path_planner.start_x=int(self.edit_x_start.text())
				self.edit_y_start.setText(str(y))
				path_planner.start_y=int(self.edit_y_start.text())
				
			if self.checkBox_finish.isChecked():
				x = event.pos().x()
				y = event.pos().y() 
				self.edit_x_finish.setText(str(x))
				path_planner.finish_x=int(self.edit_x_finish.text())
				self.edit_y_finish.setText(str(y))
				path_planner.finish_y=int(self.edit_y_finish.text())
			else:
				x = event.pos().x()
				y = event.pos().y() 
				self.edit_x_start.setText(str(x))
				self.edit_y_start.setText(str(y))
			    
		def controlTimer(self):
			# if timer is stopped
			if not self.timer.isActive():
				#self.cap = cv2.VideoCapture(1)
				self.cap = callback_imagen.image_np
				#self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
				#self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
				self.timer.start(30) #En milisegundos, el timer toma la captura del video cada 30ms
				self.start_button.setText("Stop")
			# if timer is started
			else:
				self.timer.stop()
				#self.cap.release()
				self.start_button.setText("Start")
			
	app = qtw.QApplication([])
	appWindow = path_planner(screen)
	appWindow.show()
	sys.exit(app.exec_())

	rospy.spin()
	#cv2.destroyAllWindows()


if __name__ == '__main__':
	#ros
	path_planning_all()
	#pyqt5
	#app.exec_()
	


