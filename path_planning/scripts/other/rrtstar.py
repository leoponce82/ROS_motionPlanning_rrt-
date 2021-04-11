#!/usr/bin/env python

# rrtstar.py
# This program generates a 
# asymptotically optimal rapidly exploring random tree (RRT* proposed by Sertac Keraman, MIT) in a rectangular region.
#
# Originally written by Steve LaValle, UIUC for simple RRT in
# May 2011
# Modified by Md Mahbubur Rahman, FIU for RRT* in
# January 2016

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *

#constants
XDIM = 1280
YDIM = 720
WINSIZE = [XDIM, YDIM]
EPSILON = 100
NUMNODES = 1000
RADIUS=10
OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50)]

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
			print(pt)
			print("there is obst")
			return False 
	print("no obst")
	return True

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
	global screen
	for p in nodes:
		print('se llama my check-choose parent')
		if my_checkIntersect(p,newnode,pygame,screen) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
			
		#if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
			nn = p
	newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
	newnode.parent=nn
	return newnode,nn

def reWire(nodes,newnode,pygame,screen):
	white = 255, 240, 200
	black = 20, 20, 40
	for i in range(len(nodes)):
		p = nodes[i]
		print('se llama my check-rewire')
		if my_checkIntersect(p,newnode,pygame,screen) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
			
		#if checkIntersect(p,newnode,OBS) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
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
	
def main():
	#initialize and prepare screen
	#a=checkIntersect()
	#print(a)
	global screen
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
    	
		print("#### iteracion numero:",i)
		rand = Node(random.random()*XDIM, random.random()*YDIM)#Saca una coordenada random de X y Y
#		print("random:",rand.x,rand.y)
		nn = nodes[0]
#		print("nodo nn:",nn.x,nn.y)
        
		for p in nodes:
#			print("nodes lista:", p.x,p.y)
			if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
				#print("distancia de nodo", p.x,p.y, "a nodo rand",rand.x,rand.y, "es nenor a ","distancia de nodo", nn.x,nn.y, "a nodo rand",rand.x,rand.y)
				nn = p
				#print("nodo nn cambio a:",nn.x,nn.y)
		interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
	
		newnode = Node(interpolatedNode[0],interpolatedNode[1])
#		print("nodo new node:",newnode.x,newnode.y)
		if my_checkIntersect(nn,rand,pygame,screen):
			print('se llama my check-main')
		#if checkIntersect(nn,rand,OBS):
 #         		print("no se interseca nn con rand ni OBS")
			[newnode,nn]=chooseParent(nn,newnode,nodes);
#			print "nuevo new node:", newnode.x,newnode.y, "nuevo nn:", nn.x,nn.y
       
			nodes.append(newnode)
			pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
			nodes=reWire(nodes,newnode,pygame,screen)
			pygame.display.update()
			#print i, "    ", nodes

			for e in pygame.event.get():
				if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
					sys.exit("Leaving because you requested it.")
	drawSolutionPath(start,goal,nodes,pygame,screen)
	pygame.display.update()
# if python says run, then we should run
if __name__ == '__main__':
	main()
	running = True
	while running:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False



