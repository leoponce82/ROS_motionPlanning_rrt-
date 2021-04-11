#!/usr/bin/env python
#Author: Leandro Ponce
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 25.0
NUMNODES = 10
RADIUS=50
OBS=[(500,150,100,50)]#x,y, ancho,alto

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])#si es mayor el signo seria +1

# Return true if line segments AB and CD intersect
def checkIntersect(nodeA,nodeB,OBS):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      obs=(o[0],o[1],o[0]+o[2],o[1]+o[3])#esquinas opuestas. solo dos esquinas definen el rect xy xy
      C1=(obs[0],obs[1])
      #D1=(obs[0],obs[3])
      #C2=(obs[0],obs[1])
      C2=(obs[2],obs[1])
      C3=(obs[2],obs[3])
      #D3=(obs[2],obs[1])
      #C4=(obs[2],obs[3])
      C4=(obs[0],obs[3])
      print(C1,C2,C3,C4)
      inst1= ccw(A,C1,C4) != ccw(B,C1,C4) and ccw(A,B,C1) != ccw(A,B,C4) 
      print("inst1")
      print(ccw(A,C1,C4))
      print(ccw(B,C1,C4))
      print(ccw(A,B,C1))
      print(ccw(A,B,C4)) 
      inst2= ccw(A,C1,C2) != ccw(B,C1,C2) and ccw(A,B,C1) != ccw(A,B,C2)
      print("\ninst2")
      print(ccw(A,C1,C2))
      print(ccw(B,C1,C2))
      print(ccw(A,B,C1))
      print(ccw(A,B,C2)) 
      inst3= ccw(A,C3,C2) != ccw(B,C3,C2) and ccw(A,B,C3) != ccw(A,B,C2)
      print("\ninst3")
      print(ccw(A,C3,C2))
      print(ccw(B,C3,C2))
      print(ccw(A,B,C3))
      print(ccw(A,B,C2)) 
      inst4= ccw(A,C3,C4) != ccw(B,C3,C4) and ccw(A,B,C3) != ccw(A,B,C4)
      print("\ninst4")
      print(ccw(A,C3,C4))
      print(ccw(B,C3,C4))
      print(ccw(A,B,C3))
      print(ccw(A,B,C4)) 
      inst5= ccw(A,C2,C4) != ccw(B,C2,C4) and ccw(A,B,C2) != ccw(A,B,C4)
      print("\ninst5")
      print(ccw(A,C2,C4))
      print(ccw(B,C2,C4))
      print(ccw(A,B,C2))
      print(ccw(A,B,C4)) 
      print("xx")
      print("1",inst1)
      print("2",inst2)
      print("3",inst3)
      print("4",inst4)

      if inst1==False and inst2==False and inst3==False and inst4==False:#si esto se cumple, todo bien, no se interseca
        #print(A,B)
        #input("Press Enter to continue...")
        continue      
      else:
         return False
    return True
    
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
	A=(nodeA.x,nodeA.y)
	B=(nodeB.x,nodeB.y)
	puntos=get_line(A[0],A[1],B[0],B[1])
	#print(puntos)
	for pt in puntos:
		pt1 = A#Ax,Ay
		pt2 = B
		pt3 = pt
		if (screen.get_at((pt))) == (0, 0, 255, 255):
			print("there is obst")
			return False
		
	return True
	"""
		
		x1, x2, x3 = pt1[0], pt2[0], pt3[0] #defining each coordinate for points
		y1, y2, y3 = pt1[1], pt2[1], pt3[1]
		
		if (x2==x1):
			slope='not_defined'
			if (x3==x2):
				on_and_between = min(y1, y2) <= y3 <= max(y1, y2)
				print("vertical")
				print(on_and_between)
	
		else:
			slope = (y2 - y1) / (x2 - x1) #calculating slope of first two points
			pt3_on = (y3 - y1) == slope * (x3 - x1) #checking conditions for pt3 to be on line
			pt3_between = (min(x1, x2) <= x3 <= max(x1, x2)) and (min(y1, y2) <= y3 <= max(y1, y2))
			on_and_between = pt3_on and pt3_between
			print("not vertical")
			print(on_and_between)
	"""
		
def obsDraw(pygame,screen):
	blue=(0,0,255)
	for o in OBS: 
		pygame.draw.rect(screen,blue,o)
		pygame.display.update()
class Node:
	x = 0
	y = 0
	cost=0  
	parent=None
	def __init__(self,xcoord, ycoord):
		self.x = xcoord
		self.y = ycoord
		
def pixel(surface, color, pos):
    pygame.draw.line(surface, color, pos, (pos[0]+3,pos[1]))
    pygame.display.update()
    
def main():
	pygame.init()
	screen = pygame.display.set_mode(WINSIZE)
	pygame.display.set_caption('RRTstar')
	white = 255, 255, 255
	black = 20, 20, 40
	red=255,0,0
	screen.fill(white)
	obsDraw(pygame,screen)
	pygame.display.update()
	nodes = []
    
	#A=Node(550,100)
	A=Node(450,75)
	B=Node(550,250)
	pixel(screen, red,(A.x,A.y))
	pixel(screen, red,(B.x,B.y))
	print ("intersect",my_checkIntersect(A,B,pygame,screen))#False se intersecan
	#pygame.display.update()
	
# if python says run, then we should run
if __name__ == '__main__':
	main()
	running = True
	while running:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False

