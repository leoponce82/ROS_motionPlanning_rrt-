#!/usr/bin/env python
#Author: Leandro Ponce


import rospy
import roslib
import os
import cv2
import numpy as np

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from pygame_test import Ui_pygame_test

# import  PyQt5 modules
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc



def callback(data):
	pass


def pygame_test_ros():
		
	############################# ROS SETTINGS FOR THE NODE ####################
	rospy.init_node('pygame_test_ros', anonymous=True)
	rospy.Subscriber('/imagen_procesada', CompressedImage, callback)
	#rospy.Subscriber('usb_cam/image_raw', Image, callback)
	
	#constants
	XDIM = 640
	YDIM = 480
	WINSIZE = [XDIM, YDIM]
	EPSILON = 20.0
	NUMNODES = 400
	RADIUS=50
	OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50)]
	
	def obsDraw(pygame,screen):
		blue=(0,0,255)
		for o in OBS: 
			pygame.draw.rect(screen,blue,o)
	
	def dist(p1,p2):
		return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
	
	def step_from_to(p1,p2):
		if dist(p1,p2) < EPSILON:
			return p2
		else:
			theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
			return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)
	
	def chooseParent(nn,newnode,nodes):
		for p in nodes:
			if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
				nn = p
		newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
		newnode.parent=nn
		return newnode,nn
	
	def reWire(nodes,newnode,pygame,screen):
		white = 255, 240, 200
		black = 20, 20, 40
		for i in range(len(nodes)):
			p = nodes[i]
			if checkIntersect(p,newnode,OBS) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
				pygame.draw.line(screen,white,[p.x,p.y],[p.parent.x,p.parent.y])  
				p.parent = newnode
				p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
				nodes[i]=p  
				pygame.draw.line(screen,black,[p.x,p.y],[newnode.x,newnode.y])                    
		return nodes
	
	def drawSolutionPath(start,goal,nodes,pygame,screen):
		pink = 200, 20, 240
		nn = nodes[0]
		for p in nodes:
			if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
				nn = p
		while nn!=start:
			pygame.draw.line(screen,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],2)  
			nn=nn.parent
	
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
			self.x = xcoord
			self.y = ycoord
		
	pygame.init()
	screen = pygame.display.set_mode(WINSIZE)
	#pygame.display.set_caption('RRTstar')
	#white = 255, 255, 255
	#black = 20, 20, 40
	#screen.fill(white)
	#obsDraw(pygame,screen)
	nodes = []
		    
	#nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
	nodes.append(Node(0.0,0.0)) # Start in the corner
	start=nodes[0]
	goal=Node(630.0,100.0)
	
	###################################### APP WINDOW PYQT5 #####################################################
	
	class pygame_test(qtw.QMainWindow,Ui_pygame_test): #EL PRIMER ARGUMENTO DEBE COINCIDIR CON LO ESCOGIDO EN DESIGNER
		def __init__(self,screen):
			super(pygame_test,self).__init__()
			self.setupUi(self)
			w=screen.get_width()
			h=screen.get_height()
			self.data=screen.get_buffer().raw
			self.image=QImage(self.data,w,h,QImage.Format_RGB32)
			self.start_button.clicked.connect(self.show_pygame)
			pygame.quit()#PARA ELIMINAR LA VENTANA PYGAME Q SE CREA
			
			
		def paintEvent(self,e):
			qp=QPainter()
			qp.begin(self)
			qp.drawImage(100,10,self.image)
			qp.end()
		########################################
		def show_pygame(self):
		
			#s=pygame.Surface((640,480))
			#s.fill((0,100,255))
			#pygame.draw.circle(s,(255,100,100),(200,100),25)
			#w=s.get_width()
			#h=s.get_height()
			#self.data=s.get_buffer().raw
			#self.image=QImage(self.data,w,h,QImage.Format_RGB32)
			
			#initialize and prepare screen
			#a=checkIntersect()
			#print(a)
			pygame.init()
			screen = pygame.display.set_mode(WINSIZE)
			pygame.display.set_caption('RRTstar')
			white = 255, 255, 255
			black = 20, 20, 40
			screen.fill(white)
			obsDraw(pygame,screen)
			nodes = []
		    	
			#nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
			nodes.append(Node(0.0,0.0)) # Start in the corner
			start=nodes[0]
			goal=Node(630.0,100.0)
			for i in range(NUMNODES):
		    	
				#print("#### iteracion numero:",i)
				rand = Node(random.random()*XDIM, random.random()*YDIM)#Saca una coordenada random de X y Y
				#print("random:",rand.x,rand.y)
				nn = nodes[0]
				#print("nodo nn:",nn.x,nn.y)
		        
				for p in nodes:
					#print("nodes lista:", p.x,p.y)
					if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
						#print("distancia de nodo", p.x,p.y, "a nodo rand",rand.x,rand.y, "es nenor a ","distancia de nodo", nn.x,nn.y, "a nodo rand",rand.x,rand.y)
						nn = p
						#print("nodo nn cambio a:",nn.x,nn.y)
				interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
			
				newnode = Node(interpolatedNode[0],interpolatedNode[1])
				#print("nodo new node:",newnode.x,newnode.y)
				if checkIntersect(nn,rand,OBS):
		          		#print("no se interseca nn con rand ni OBS")
					[newnode,nn]=chooseParent(nn,newnode,nodes);
					#print "nuevo new node:", newnode.x,newnode.y, "nuevo nn:", nn.x,nn.y
		       
					nodes.append(newnode)
					pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
					nodes=reWire(nodes,newnode,pygame,screen)
					pygame.display.update()
		
					for e in pygame.event.get():
						if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
							sys.exit("Leaving because you requested it.")
			
			drawSolutionPath(start,goal,nodes,pygame,screen)
			pygame.display.update()
			w=screen.get_width()
			h=screen.get_height()
			self.data=screen.get_buffer().raw
			self.image=QImage(self.data,w,h,QImage.Format_RGB32)
			#pygame.quit()#PARA ELIMINAR LA VENTANA PYGAME Q SE CREA
			
	app = qtw.QApplication([])
	appWindow = pygame_test(screen)
	appWindow.show()
	sys.exit(app.exec_())
	rospy.spin()
	#cv2.destroyAllWindows()


if __name__ == '__main__':
	#ros
	pygame_test_ros()
	#pyqt5
	#app.exec_()

