#!/usr/bin/env python
import PID # tomado de https://github.com/ivmech/ivPID PID DISENIADO POR ivPID 
import multiprocessing 
from multiprocessing import Queue
import sys
import serial
import struct
import binascii
import rospy
import json
import numpy as np
import time
import math
# import  PyQt5 modules
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc

from control import Ui_control

from math import sqrt,cos,sin,atan2
from collisions_monitor.msg import colision_msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from cam_test.msg import IntList
from cam_test.msg import ListArray
from cv2 import aruco
import os
import time
import cv2
from time import sleep
from xbee import XBee,ZigBee

font = cv2.FONT_HERSHEY_COMPLEX

R=16#mm
L=50#mm
velocidad_cte=0 #unidades?
xbee = None
# xbee address DH + DL concatenated
#router4 0013A2004191C5FB
#XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\xC5\xFB'#New car, 4th car
# xbee 16-bit Source Address MY
#XBEE_ADDR_SHORT= '\xFF\xFE' # Carro numero 4 My=4
# port number
DEVICE = '/dev/ttyUSB0'
ser = serial.Serial(DEVICE, 57600)
xbee = ZigBee(ser)

aruco_centerback=0
angulos=[]
angulos_global=[]
waypoints4=[]
waypoints3=[]
waypoints2=[]
waypoints1=[]
waypoints_global4=[]
waypoints_global3=[]
waypoints_global2=[]
waypoints_global1=[]
waypoints_list4=[]
waypoints_list3=[]
waypoints_list2=[]
waypoints_list1=[]
all_waypoints_list=[]
ids=[]
ids_global=[]
aruco=[]
aruco_global=[]
aruco_aux=[]
ids_aux=[]
angulos_aux=[]
id_colision=0
cmd_stop=False #True si es q debe detenerse

target_angle = 35
#P = 0.15
#I = 0.005
#D = 0.02
#pid = PID.PID(P, I, D)
#pid.SetPoint = target_angle
#pid.setSampleTime(0.1)

def aruco_center_calc(controlled_aruco_corners):
	center = [(controlled_aruco_corners[0]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[5])/2]
	return center

def read_aruco():
	global aruco
	global aruco_aux
	global aruco_global
	try:
		with open('/home/proyinv/ponce_ws/src/cam_test/scripts/coord_obstaculos_aruco.json') as f:#desde mapa2d
			data_json = json.load(f)
		n=0
		aruco=[]
		aruco_aux=[]
		aruco_global=[]
		for item in data_json['aruco']:
			index=item['index']
			coord=item['coord']
			aruco.append(coord)
			n=n+1
		aruco_global=list(aruco)
		aruco_aux=aruco_global
		return aruco_global
	except ValueError:
		return aruco_aux

def read_ids():
	global ids
	global ids_aux
	global ids_global
	try:
		with open('/home/proyinv/ponce_ws/src/cam_test/scripts/coord_obstaculos_aruco.json') as f:#desde mapa2d
			data_json = json.load(f)
		n=0
		ids=[]
		ids_aux=[]
		ids_global=[]
		for item in data_json['ids']:
			index=item['index']
			id_aruco=item['id']
			ids.append(id_aruco)
			n=n+1
		ids_global=list(ids)
		ids_aux=ids_global
		return ids_global
	except ValueError:
		return ids_aux

def read_angle():
	global angulos
	global angulos_aux
	global angulos_global
	try:
		with open('/home/proyinv/ponce_ws/src/cam_test/scripts/coord_obstaculos_aruco.json') as f:#desde mapa2d
			data_json = json.load(f)
		n=0
		angulos=[]
		angulos_aux=[]
		angulos_global=[]
		for item in data_json['angulos']:
			index=item['index']
			aruco_angulo=item['angulo']
			angulos.append(aruco_angulo)
			n=n+1
		angulos_global=list(angulos)
		angulos_aux=angulos_global
		return angulos_global
	except ValueError:
		return angulos_aux

def dist(p1,p2):#p1 y p2 son coordenadas tipo [x,y]
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
	
def callback_waypoints(data):
	global all_waypoints_list
	#global end
	os.system('clear')
	print("######### waypoint update #########")
	print(data)
	with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json') as f:
		data_json = json.load(f)
	n=0
	waypoints4=[]
	global waypoints_list4
	for item in data_json['waypoints4']:
		index=item['index']
		coord=item['coord']
		waypoints4.append(coord)
		n=n+1
	waypoints_list4=list(waypoints4)
	
	n=0
	waypoints3=[]
	global waypoints_list3
	for item in data_json['waypoints3']:
		index=item['index']
		coord=item['coord']
		waypoints3.append(coord)
		n=n+1
	waypoints_list3=list(waypoints3)
	
	n=0
	waypoints2=[]
	global waypoints_list2
	for item in data_json['waypoints2']:
		index=item['index']
		coord=item['coord']
		waypoints2.append(coord)
		n=n+1
	waypoints_list2=list(waypoints2)
	
	n=0
	waypoints1=[]
	global waypoints_list1
	for item in data_json['waypoints1']:
		index=item['index']
		coord=item['coord']
		waypoints1.append(coord)
		n=n+1
	waypoints_list1=list(waypoints1)
	
	all_waypoints_list=[[0],[waypoints_list1],[waypoints_list2],[waypoints_list3],[waypoints_list4]]
	
	#print(waypoints_list)
def callback_colisiones(data):
	global id_colision,cmd_stop
	global queue
	#print(data)
	id_colision_list=data.aruco_id
	#cmd_stop=data.cmd_stop #True si es q debe detenerse
	queue.put(id_colision_list)
	#print("colisiones callback",stop_array) 



def callback_imagen(data):
	
	br = CvBridge()
	np_arr = np.fromstring(data.data, np.uint8)
	callback_imagen.image_np = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR)
	colors_list=[(0, 255, 0),(255, 0, 0),(0, 0, 255),(255, 0, 255)]
	
	for each_list in all_waypoints_list:
		if each_list[0]!=0:
			for n in range(len(each_list[0])):
				line_thickness = 2
				if n<( (len(each_list[0]))-1 ):
					cv2.line(callback_imagen.image_np, (int(each_list[0][n][0]), int(each_list[0][n][1])), (int(each_list[0][n+1][0]), int(each_list[0][n+1][1])), colors_list[all_waypoints_list.index(each_list)-1], thickness=line_thickness)
					cv2.circle(callback_imagen.image_np,(int(each_list[0][n][0]), int(each_list[0][n][1])), 2, (255,255,255), -1)


	
	################# DIBUJA CENTROS  de ARUCO #############
	
	for i in range(4):
		try:
			controlled_index = read_ids().index([i+1])
			angulo_actual=read_angle()[controlled_index]
			controlled_aruco_corners=read_aruco()[controlled_index]
			aruco_center=aruco_center_calc(controlled_aruco_corners) #centro del aruco controlado, [x,y]
			aruco_centerpoint=[(controlled_aruco_corners[0]+controlled_aruco_corners[2])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[3])/2] # el centerpoint representaria la bolaloca
			aruco_centerback=[(controlled_aruco_corners[6]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[7]+controlled_aruco_corners[5])/2] # centerback el punto medio del eje de ruedas
			aruco_centerback=aruco_centerback


			a=-40;
			phi=math.radians(angulo_actual)
			xr=aruco_centerpoint[0]+a*cos(phi);
			yr=aruco_centerpoint[1]+a*sin(-phi);
			x=xr
			y=yr
			a_point=(int(xr), int(yr))
			###################################
			aruco_centerback_draw=tuple ( (int(aruco_centerback[0]),int(aruco_centerback[1])) )
			a_point=(int(x), int(y))
			cv2.circle(callback_imagen.image_np,aruco_centerback_draw, 2, (0,0,255), -1)
			cv2.circle(callback_imagen.image_np,a_point, 2, (0,255,0), -1)
		except ValueError:
			#print('No se ha detectado aruco ID_ IMAGEN PROCESS',i+1)
			pass
		
	#cv2.imshow("camera_route",callback_imagen.image_np)
	#cv2.waitKey(1)

	
def position_control_all():
	global all_waypoints_list
	global id_colision,cmd_stop
	global queue
	
	queue=Queue()
	
	rospy.init_node('position_control_all', anonymous=True)
	rospy.Subscriber('/coord_waypoints_update_indicator', String, callback_waypoints)#viene de path_planning
	rospy.Subscriber('/colisiones', colision_msg, callback_colisiones)
	#rospy.Subscriber("/coord_obstaculos_aruco", String, callback_angulos)#viene de mapa2d, mapa2d es el publisher
	rospy.Subscriber('/imagen_procesada_colision', CompressedImage, callback_imagen)
	
	print("######### first waypoint update #########")
	with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json') as f:
		data_json = json.load(f)
	n=0
	waypoints4=[]
	global waypoints_list4
	for item in data_json['waypoints4']:
		index=item['index']
		coord=item['coord']
		waypoints4.append(coord)
		n=n+1
	waypoints_list4=list(waypoints4)
	
	n=0
	waypoints3=[]
	global waypoints_list3
	for item in data_json['waypoints3']:
		index=item['index']
		coord=item['coord']
		waypoints3.append(coord)
		n=n+1
	waypoints_list3=list(waypoints3)
	
	n=0
	waypoints2=[]
	global waypoints_list2
	for item in data_json['waypoints2']:
		index=item['index']
		coord=item['coord']
		waypoints2.append(coord)
		n=n+1
	waypoints_list2=list(waypoints2)
	
	n=0
	waypoints1=[]
	global waypoints_list1
	for item in data_json['waypoints1']:
		index=item['index']
		coord=item['coord']
		waypoints1.append(coord)
		n=n+1
	waypoints_list1=list(waypoints1)
	
	all_waypoints_list=[[0],[waypoints_list1],[waypoints_list2],[waypoints_list3],[waypoints_list4]]
	
	def control_exe(id_aruco):
		global all_waypoints_list
		global aruco
		global aruco_aux
		global aruco_global
		global ids
		global ids_aux
		global ids_global
		global angulos
		global angulos_aux
		global angulos_global

	 	control_exe.ident=os.getpid()
	 	control_exe.aruco_ident=id_aruco
	 	
		if id_aruco==1:
			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\x78\x2D'#1st car 00 13 A2 00 41 91 78 2D
			XBEE_ADDR_SHORT= '\xFC\x91' 
		if id_aruco==2:
			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\xC1\x55\x52'#2d car 00 13 A2 00 41 91 C5 E1
			XBEE_ADDR_SHORT= '\xA5\xAE' 
		if id_aruco==3:
			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\xC5\xED'# 3d car 00 13 A2 00 41 91 C5 ED
			XBEE_ADDR_SHORT= '\x97\xF5' 
		if id_aruco==4:
			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\xC5\xFB'#New car, 4th car
			XBEE_ADDR_SHORT= '\x29\xED' 
	
		print(id_aruco)	
		print(all_waypoints_list)
		
		#os.system('clear')
		#print("######### coord aruco update #########")
		#if (not len(all_waypoints_list)==False): #True si es q la lista esta vacia
	
		for each_coord in all_waypoints_list[id_aruco][0]:
			#each_coord=[531.0,397.0]
			#os.system('clear')
			print('inicio de',each_coord)
			try:
				controlled_index = read_ids().index([id_aruco])#Para cotrolar el robot con ID requerido
			except ValueError: 
				print('No se ha detectado el robot con ID=',id_aruco)
				break
			angulo_actual=read_angle()[controlled_index]
			controlled_aruco_corners=read_aruco()[controlled_index]
			aruco_center=aruco_center_calc(controlled_aruco_corners) #centro del aruco controlado, [x,y]
			aruco_centerpoint=[(controlled_aruco_corners[0]+controlled_aruco_corners[2])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[3])/2] # el centerpoint representaria la bolaloca
			aruco_centerback=[(controlled_aruco_corners[6]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[7]+controlled_aruco_corners[5])/2] # centerback el punto medio del eje de ruedas
			control_exe.aruco_centerback=aruco_centerback
			vectorx_aruco=aruco_centerpoint[0]-aruco_centerback[0]
			vectory_aruco=aruco_centerpoint[1]-aruco_centerback[1]
			vectorx=each_coord[0]
			vectory=each_coord[1]
			vectorx_new=vectorx-aruco_centerback[0]
			vectory_new=vectory-aruco_centerback[1]
			
			angulo_deseado_grados=-math.degrees(math.atan2(vectory_new, vectorx_new))
			angulo_aruco_atan=math.degrees(math.atan2(vectory_aruco, vectorx_aruco))
			error=angulo_deseado_grados - angulo_actual
			
			angulo_actual_360=angulo_actual
			if (angulo_actual_360 < 0):
				angulo_actual_360 += 360; #tiene rango de 0 a 360
			
			angulo_deseado_grados_360=angulo_deseado_grados
			if (angulo_deseado_grados_360 < 0):
				angulo_deseado_grados_360 += 360; #tiene rango de 0 a 360
			error_360 = angulo_deseado_grados_360 - angulo_actual_360
			
			error_360=error_360*-1
			error=error*-1
			
			if abs(error) <= abs(error_360):

				angulo_actual_pid=angulo_actual
				angulo_deseado_grados_pid=angulo_deseado_grados
				P = 0.1
				I = 0.0
				D = 0.0
				pid = PID.PID(P, I, D)
				pid.setSampleTime(0.001)
				cual='rango_180'
			else:

				angulo_actual_pid=angulo_actual_360
				angulo_deseado_grados_pid=angulo_deseado_grados_360
				P = 0.1
				I = 0.0
				D = 0.0
				pid = PID.PID(P, I, D)
				pid.setSampleTime(0.001)
				cual='rango_360'
				
			"""
			if (angulo_deseado_grados > 180):
				angulo_deseado_grados = angulo_deseado_grados-360;
			if angulo_deseado_grados <= -180:
				angulo_deseado_grados = angulo_deseado_grados+360;
			
			
			if (error > 180):
				error = error-360;
			if error <= -180:
				error = error+360;
			"""
			
			
			a=-40
			phi=math.radians(angulo_actual)
			xr=aruco_centerpoint[0]+a*cos(phi);
			yr=aruco_centerpoint[1]+a*sin(-phi);
			control_exe.x=xr
			control_exe.y=yr
			a_point=(int(xr), int(yr))
			

			#while (   dist(aruco_center,each_coord) > 5    ):
			while ( dist(a_point,each_coord) > 10 ):
			
				if cual=='rango_180':
			
					if abs(error)>6 :
						os.system('clear')
						P = 0.1
						I = 0.00
						D = 0.00
						pid = PID.PID(P, I, D)
						print(each_coord)
						print("only rotate 180")
						velocidad_cte=0 #unidades?
		
						pid.SetPoint = angulo_deseado_grados_pid
						pid.update(angulo_actual_pid)
						velocidad_w = pid.output
						list_to_stop=queue.get()
						for each_id in list_to_stop:
							if each_id==id_aruco:
								vr=0
								vl=0
								print("STOP")
								break
							else:
								vr=int(round( ( (2*velocidad_cte+velocidad_w*L)/(2*R) )))
								vl=int(round( ( (2*velocidad_cte-velocidad_w*L)/(2*R) )))
				
			
						if vr >= 16:
							vr=16
						if vr <= -16:
							vr=-16
						if vl >= 16:
							vl=16
						if vl <= -16:
							vl=-16
						data1 = struct.pack('hhh',2,vr,vl) #2 es para setear velocidades
						xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
						#rospy.loginfo("Estructura del dato Robot1: %s" % binascii.hexlify(data1))
						print("vel r:",vr)
						print("vel l:",vl)
					else:	
						os.system('clear')
						P = 0.1
						I = 0.00
						D = 0.00
						pid = PID.PID(P, I, D)
						print("traverse")
						velocidad_cte=600 #unidades?
				
						pid.SetPoint = angulo_deseado_grados_pid
						pid.update(angulo_actual_pid)
						velocidad_w = pid.output
						list_to_stop=queue.get()
						for each_id in list_to_stop:
							if each_id==id_aruco:
								vr=0
								vl=0
								print("STOP")
								break
							else:
								vr=int(round( ( (2*velocidad_cte+velocidad_w*L)/(2*R) )))
								vl=int(round( ( (2*velocidad_cte-velocidad_w*L)/(2*R) )))
			
						if vr >= 50:
							vr=50
						if vr <= -50:
							vr=-50
						if vl >= 50:
							vl=50
						if vl <= -50:
							vl=-50
						data1 = struct.pack('hhh',2,vr,vl) #2 es para setear velocidades
						xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
						#rospy.loginfo("Estructura del dato Robot1: %s" % binascii.hexlify(data1))
						print("vel r:",vr)
						print("vel l:",vl)
				
				elif cual=='rango_360':
				
					if abs(error_360)>6 :
						os.system('clear')
						P = 0.1
						I = 0.00
						D = 0.00
						pid = PID.PID(P, I, D)
						print(each_coord)
						print("only rotate 360")
						velocidad_cte=0 #unidades?
					
						pid.SetPoint = angulo_deseado_grados_pid
						pid.update(angulo_actual_pid)
						velocidad_w = pid.output
						list_to_stop=queue.get()
						for each_id in list_to_stop:
							if each_id==id_aruco:
								vr=0
								vl=0
								print("STOP")
								break
							else:
								vr=int(round( ( (2*velocidad_cte+velocidad_w*L)/(2*R) )))
								vl=int(round( ( (2*velocidad_cte-velocidad_w*L)/(2*R) )))
				
						if vr >= 16:
							vr=16
						if vr <= -16:
							vr=-16
						if vl >= 16:
							vl=16
						if vl <= -16:
							vl=-16
						data1 = struct.pack('hhh',2,vr,vl) #2 es para setear velocidades
						xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
						#rospy.loginfo("Estructura del dato Robot1: %s" % binascii.hexlify(data1))
						print("vel r:",vr)
						print("vel l:",vl)
					else:	
						os.system('clear')
						P = 0.1
						I = 0.00
						D = 0.00
						pid = PID.PID(P, I, D)
						print(each_coord)
						print("traverse")
						velocidad_cte=600 #unidades?
				
						pid.SetPoint = angulo_deseado_grados_pid
						pid.update(angulo_actual_pid)
						velocidad_w = pid.output
						list_to_stop=queue.get()
						for each_id in list_to_stop:
							if each_id==id_aruco:
								vr=0
								vl=0
								print("STOP")
								break
							else:
								vr=int(round( ( (2*velocidad_cte+velocidad_w*L)/(2*R) )))
								vl=int(round( ( (2*velocidad_cte-velocidad_w*L)/(2*R) )))
				
						if vr >= 50:
							vr=50
						if vr <= -50:
							vr=-50
						if vl >= 50:
							vl=50
						if vl <= -50:
							vl=-50
						data1 = struct.pack('hhh',2,vr,vl) #2 es para setear velocidades
						xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
						#rospy.loginfo("Estructura del dato Robot1: %s" % binascii.hexlify(data1))
						print("vel r:",vr)
						print("vel l:",vl)
		
				controlled_index = read_ids().index([id_aruco])#Para cotrolar el robot con ID 1
				angulo_actual=read_angle()[controlled_index] # tiene rango de -180 a 180
				controlled_aruco_corners=read_aruco()[controlled_index]
				aruco_center=aruco_center_calc(controlled_aruco_corners) #centro del aruco controlado, [x,y]
				aruco_centerpoint=[(controlled_aruco_corners[0]+controlled_aruco_corners[2])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[3])/2] 
				aruco_centerback=[(controlled_aruco_corners[6]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[7]+controlled_aruco_corners[5])/2] 
				control_exe.aruco_centerback=aruco_centerback
				vectorx_aruco=aruco_centerpoint[0]-aruco_centerback[0]
				vectory_aruco=aruco_centerpoint[1]-aruco_centerback[1]
				vectorx=each_coord[0]
				vectory=each_coord[1]
				vectorx_new=vectorx-aruco_centerback[0]
				vectory_new=vectory-aruco_centerback[1]
				
				angulo_deseado_grados=-math.degrees(math.atan2(vectory_new, vectorx_new))
				angulo_aruco_atan=math.degrees(math.atan2(vectory_aruco, vectorx_aruco))
				error=angulo_deseado_grados - angulo_actual
			
				angulo_actual_360=angulo_actual
				if (angulo_actual_360 < 0):
					angulo_actual_360 += 360; #tiene rango de 0 a 360
			
				angulo_deseado_grados_360=angulo_deseado_grados
				if (angulo_deseado_grados_360 < 0):
					angulo_deseado_grados_360 += 360; #tiene rango de 0 a 360
				error_360 = angulo_deseado_grados_360 - angulo_actual_360
			
				error_360=error_360*-1
				error=error*-1
			
				if abs(error) <= abs(error_360):

					angulo_actual_pid=angulo_actual
					angulo_deseado_grados_pid=angulo_deseado_grados
					P = 0.1
					I = 0.00
					D = 0.00
					pid = PID.PID(P, I, D)
					pid.setSampleTime(0.001)
					cual='rango_180'
				else:
	
					angulo_actual_pid=angulo_actual_360
					angulo_deseado_grados_pid=angulo_deseado_grados_360
					P = 0.1
					I = 0.00
					D = 0.00
					pid = PID.PID(P, I, D)
					pid.setSampleTime(0.001)
					cual='rango_360'
				
				
			
					
				a=-40
				phi=math.radians(angulo_actual)
				xr=aruco_centerpoint[0]+a*cos(phi);
				yr=aruco_centerpoint[1]+a*sin(-phi);
				control_exe.x=xr
				control_exe.y=yr
				a_point=(int(xr), int(yr))
				
				
				
			print('finished waypoint:', each_coord)
			data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
			xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
			#break
		
		print('Se terminaron los waypoints')
		data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
		xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
		data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
		xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
		data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
		xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
		print("Stopped")
		return
	
	"""	
		
	else:
		print('Waiting for waypoints')
		data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
		xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
	"""	
	
	def Multifunc(lista):
		ps = []
		"""
		if len(lista)==1:
			p = multiprocessing.Process(target=control_exe , args=(lista[0],))
			ps.append(p)
			p.start()
		if len(lista)>1:
			for i in lista:
				p = multiprocessing.Process(target=control_exe , args=(i,))
				ps.append(p)
				p.start()
		"""
		for i in lista:
			p = multiprocessing.Process(target=control_exe , args=(i,))
			ps.append(p)
			p.start()
		return ps
	
	##############################################################################################
	class control(qtw.QMainWindow,Ui_control): #EL PRIMER ARGUMENTO DEBE COINCIDIR CON LO ESCOGIDO EN DESIGNER
		
		
		def __init__(self, *args, **kwargs):
			super(control,self).__init__(*args, **kwargs)
			self.setupUi(self)
			self.timer = QTimer()
			self.timer.timeout.connect(self.verCam)
			self.start_button.clicked.connect(self.controlTimer)
			
			self.start_button1.clicked.connect(self.ejecutar_control1)
			self.start_button2.clicked.connect(self.ejecutar_control2)
			self.start_button3.clicked.connect(self.ejecutar_control3)
			self.start_button4.clicked.connect(self.ejecutar_control4)
			self.start_button_all.clicked.connect(self.ejecutar_control0) #se emite 0 para controlar todos al mismo tiempo
			self.stop_button.clicked.connect(self.stop_all) #se emite 0 para controlar todos al mismo tiemp
			
			
		def ejecutar_control1(self):
			
			if self.start_button1.text() == "Start 1":
				print("Started")
				self.start_button1.setText("Stop 1")
				self.ps = Multifunc([1])
				return
			else:	
				print("Stop sent")
				#print(control_exe.ident)
				for p in self.ps:
#		 			if control_exe.aruco_ident==1:
#		 				os.kill(control_exe.ident)
            				p.terminate()
            				print("Stop sent")
            			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\x78\x2D'#1st car 00 13 A2 00 41 91 78 2D
				XBEE_ADDR_SHORT= '\xFC\x91'
				data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
				xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
				self.start_button1.setText("Start 1")
					
		
			
		def ejecutar_control2(self):
			if self.start_button2.text() == "Start 2":
				print("Started")
				self.start_button2.setText("Stop 2")
				self.ps = Multifunc([2])
				return
			else:
				
				print("Stop sent")
				for p in self.ps:
            				p.terminate()
            				print("Stop sent")
            			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\xC1\x55\x52'#2d car 00 13 A2 00 41 91 C5 E1
				XBEE_ADDR_SHORT= '\xA5\xAE' 
				data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
				xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
				self.start_button2.setText("Start 2")
				
		def ejecutar_control3(self):
			if self.start_button3.text() == "Start 3":
				print("Started")
				self.start_button3.setText("Stop 3")
				self.ps = Multifunc([3])
				return
			else:
				print("Stop sent")
				for p in self.ps:
            				p.terminate()
            				print("Stop sent")
            			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\xC5\xED'# 3d car 00 13 A2 00 41 91 C5 ED
				XBEE_ADDR_SHORT= '\x97\xF5'
				data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
				xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
				self.start_button3.setText("Start 3")
				
		def ejecutar_control4(self):
			if self.start_button4.text() == "Start 4":
				print("Started")
				self.start_button4.setText("Stop 4")
				self.ps = Multifunc([4])
				return
			else:
				print("Stop sent")
				for p in self.ps:
            				p.terminate()
            				print("Stop sent")
            			XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\xC5\xFB'#New car, 4th car
				XBEE_ADDR_SHORT= '\x29\xED' 
				data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
				xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
				self.start_button4.setText("Start 4")
		
		def ejecutar_control0(self):
			if self.start_button_all.text() == "Start all":
				print("Started all")
				self.start_button_all.setText("Stop all")
				self.ps = Multifunc([1,2,3,4])
				return
			else:
				print("Stop sent")
				for p in self.ps:
            				p.terminate()
				self.start_button_all.setText("Start all")
				
		def stop_all(self):#no funciona :(
			t_end = time.time() + 1
			while time.time() < t_end:
				for p in self.ps:
		    			p.terminate()
		    			#print("Stop sent")
			

		# ver camara
		def verCam(self):
			image = callback_imagen.image_np
			image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
			height, width, channel = image.shape # Saca la informacion de la imagen
			step = channel * width
			qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)#Formato para mostrar en pyqt5
			self.label_video.setPixmap(QPixmap.fromImage(qImg))# Se muestra en label_video con Qpixmap

		def controlTimer(self):
			# if timer is stopped
			if not self.timer.isActive():
				self.cap = callback_imagen.image_np
				self.timer.start(30) #En milisegundos, el timer toma la captura del video cada 30ms
				self.start_button.setText("Stop")
			# if timer is started
			else:
				self.timer.stop()
				self.start_button.setText("Start")


			
	app = qtw.QApplication([])
	appWindow = control()
	appWindow.show()
	sys.exit(app.exec_())
	rospy.spin()
	#cv2.destroyAllWindows()
	##############################################################################################	

if __name__ == '__main__':
	position_control_all()
