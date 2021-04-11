#!/usr/bin/env python
import PID # tomado de https://github.com/ivmech/ivPID PID DISENIADO POR ivPID 
import serial
import struct
import binascii
import rospy
import json
import numpy as np
import time
import math
from math import sqrt,cos,sin,atan2
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
import cv2
from time import sleep
from xbee import XBee,ZigBee

font = cv2.FONT_HERSHEY_COMPLEX

R=16#mm
L=50#mm
velocidad_cte=0 #unidades?
xbee = None
# xbee address DH + DL concatenated
#router1 0013A2004191C5FB
XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x41\x91\xC5\xFB'#New car, 4th car
# xbee 16-bit Source Address MY
XBEE_ADDR_SHORT= '\xFF\xFE' # Carro numero 4 My=4
# port number
DEVICE = '/dev/ttyUSB1'
ser = serial.Serial(DEVICE, 57600)
xbee = ZigBee(ser)

aruco_centerback=0
angulos=[]
angulos_global=[]
waypoints=[]
waypoints_global=[]
waypoints_list=[]
ids=[]
ids_global=[]
aruco=[]
aruco_global=[]
aruco_aux=[]
ids_aux=[]
angulos_aux=[]

target_angle = 35
P = 0.07
I = 0
D = 0
pid = PID.PID(P, I, D)
pid.SetPoint = target_angle
pid.setSampleTime(0.1)

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
	#global end
	os.system('clear')
	print("######### waypoint update #########")
	print(data)
	with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json') as f:
		data_json = json.load(f)
	n=0
	waypoints=[]
	global waypoints_list
	for item in data_json['waypoints4']:
		index=item['index']
		coord=item['coord']
		waypoints.append(coord)
		n=n+1
	waypoints_list=list(waypoints)
	callback_waypoints.end=False#False cuando aun no se ha terminado con los waypoints
	#print(waypoints_list)
	
def callback_angulos(data):
	global aruco
	global aruco_aux
	global aruco_global
	global ids
	global ids_aux
	global ids_global
	global angulos
	global angulos_aux
	global angulos_global
	#global end
	os.system('clear')
	#print("######### coord aruco update #########")
	#print(data)
	if (not len(waypoints_list)==False) and callback_waypoints.end==False: #True si es q la lista esta vacia
		for each_coord in waypoints_list:
			#each_coord=[531.0,397.0]
			#os.system('clear')
			print('inicio de',each_coord)
			try:
				controlled_index = read_ids().index([4])#Para cotrolar el robot con ID 4
			except ValueError: 
				print('No se ha detectado el robot con ID=4')
				break
			angulo_actual=read_angle()[controlled_index]
			controlled_aruco_corners=read_aruco()[controlled_index]
			aruco_center=aruco_center_calc(controlled_aruco_corners) #centro del aruco controlado, [x,y]
			aruco_centerpoint=[(controlled_aruco_corners[0]+controlled_aruco_corners[2])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[3])/2] # el centerpoint representaria la bolaloca
			aruco_centerback=[(controlled_aruco_corners[6]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[7]+controlled_aruco_corners[5])/2] # centerback el punto medio del eje de ruedas
			callback_angulos.aruco_centerback=aruco_centerback
			vectorx_aruco=aruco_centerpoint[0]-aruco_centerback[0]
			vectory_aruco=aruco_centerpoint[1]-aruco_centerback[1]
			vectorx=each_coord[0]
			vectory=each_coord[1]
			vectorx_new=vectorx-aruco_centerback[0]
			vectory_new=vectory-aruco_centerback[1]
			angulo_output = math.atan2(vectory_new, vectorx_new) - math.atan2(vectory_aruco, vectorx_aruco);
			angulo_output_grados=math.degrees(angulo_output)
			angulo_output_grados=angulo_output_grados*-1
			error=angulo_output_grados
			if (error > 180):
				error = error-360;
			if error <= -180:
				error = error+360;
				
			angulo_deseado_grados=error+angulo_actual
			
			if (angulo_deseado_grados > 180):
				angulo_deseado_grados = angulo_deseado_grados-360;
			if angulo_deseado_grados <= -180:
				angulo_deseado_grados = angulo_deseado_grados+360;
				
			a=-30;
			phi=math.radians(angulo_actual)
			xr=aruco_centerpoint[0]+a*cos(phi);
			yr=aruco_centerpoint[1]+a*sin(-phi);
			callback_angulos.x=xr
			callback_angulos.y=yr
			a_point=(int(xr), int(yr))

			xre = each_coord[0] - xr
			yre = each_coord[1] - yr
			e=np.array([[xre],[yre]])

			
			#while (   dist(aruco_center,each_coord) > 5    ):
			while ( dist(aruco_center,each_coord) > 10 ):
				#os.system('clear')
				"""
				#CONTROL OF ORIENTATION
				print(each_coord)
				print("follow?")
				velocidad_cte=200 #unidades?
				print("angulo_deseado_grados:",angulo_deseado_grados)
				print("error:",error)
				pid.SetPoint = angulo_deseado_grados
				pid.update(angulo_actual)
				velocidad_w = pid.output
				print("velocidad_w:",velocidad_w)
				"""
				
				# Matriz Jacobiana
				J=np.array([[cos(phi),-a*sin(-phi)],[sin(-phi),a*cos(phi)]])
				K=np.array([[5, 0],[0,5]])
				invJ = np.linalg.inv(J)
				v = np.dot( np.dot(invJ,K) , e)
				velocidad_cte=v[0][0]
				velocidad_w=v[1][0]
				vr=int(round( ( (2*velocidad_cte+velocidad_w*L)/(2*R) )))
				vl=int(round( ( (2*velocidad_cte-velocidad_w*L)/(2*R) )))
				#vr=16
				#vl=-16
				if vr >= 32767:
					vr=32767
				if vr <= -32767:
					vr=-32767
				if vl >= 32767:
					vl=32767
				if vl <= -32767:
					vl=-32767
				data1 = struct.pack('hhh',2,vr,vl) #2 es para setear velocidades
				xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
				#rospy.loginfo("Estructura del dato Robot1: %s" % binascii.hexlify(data1))
				print("vel r:",vr)
				print("vel l:",vl)
		
				controlled_index = read_ids().index([4])#Para cotrolar el robot con ID 1
				angulo_actual=read_angle()[controlled_index]
				controlled_aruco_corners=read_aruco()[controlled_index]
				aruco_center=aruco_center_calc(controlled_aruco_corners) #centro del aruco controlado, [x,y]
				aruco_centerpoint=[(controlled_aruco_corners[0]+controlled_aruco_corners[2])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[3])/2] 
				aruco_centerback=[(controlled_aruco_corners[6]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[7]+controlled_aruco_corners[5])/2] 
				callback_angulos.aruco_centerback=aruco_centerback
				vectorx_aruco=aruco_centerpoint[0]-aruco_centerback[0]
				vectory_aruco=aruco_centerpoint[1]-aruco_centerback[1]
				vectorx=each_coord[0]
				vectory=each_coord[1]
				vectorx_new=vectorx-aruco_centerback[0]
				vectory_new=vectory-aruco_centerback[1]
				angulo_output = math.atan2(vectory_new, vectorx_new) - math.atan2(vectory_aruco, vectorx_aruco);
				angulo_output_grados=math.degrees(angulo_output)
				angulo_output_grados=angulo_output_grados*-1
				error=angulo_output_grados
				if (error > 180):
					error = error-360;
				if error <= -180:
					error = error+360;
				angulo_deseado_grados=error+angulo_actual
				if (angulo_deseado_grados > 180):
					angulo_deseado_grados = angulo_deseado_grados-360;
				if angulo_deseado_grados <= -180:
					angulo_deseado_grados = angulo_deseado_grados+360;
				a=-30;
				phi=math.radians(angulo_actual)
				xr=aruco_centerpoint[0]+a*cos(phi);
				yr=aruco_centerpoint[1]+a*sin(-phi);
				callback_angulos.x=xr
				callback_angulos.y=yr
				a_point=(int(xr), int(yr))

				xre = each_coord[0] - xr
				yre = each_coord[1] - yr
				e=np.array([[xre],[yre]])
			
			
			print('finished waypoint:', each_coord)
			data1 = struct.pack('hhh',2,0,0) #2 es para setear velocidades
			xbee.tx(dest_addr_long = XBEE_ADDR_LONG, dest_addr = XBEE_ADDR_SHORT, data=data1,)
			#break
			
		print('Se terminaron los waypoints')
		callback_waypoints.end=True
		
		
		
	else:
		print('Waiting for waypoints')
		

def callback_imagen(data):
	br = CvBridge()
	np_arr = np.fromstring(data.data, np.uint8)
	callback_imagen.image_np = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR)
	if (not len(waypoints_list)==False): #verifica si la lista esta vacia o no, si no esta vacia la dibuja
		n=0
		for n in range(len(waypoints_list)):
			line_thickness = 2
			if n<( (len(waypoints_list))-1 ):
				cv2.line(callback_imagen.image_np, (int(waypoints_list[n][0]), int(waypoints_list[n][1])), (int(waypoints_list[n+1][0]), int(waypoints_list[n+1][1])), (0, 255, 0), thickness=line_thickness)
				cv2.circle(callback_imagen.image_np,(int(waypoints_list[n][0]), int(waypoints_list[n][1])), 2, (255,0,0), -1)
	try:
		aruco_centerback_draw=tuple ( (int(callback_angulos.aruco_centerback[0]),int(callback_angulos.aruco_centerback[1])) )
		a_point=(int(callback_angulos.x), int(callback_angulos.y))
		cv2.circle(callback_imagen.image_np,aruco_centerback_draw, 2, (0,0,255), -1)
		cv2.circle(callback_imagen.image_np,a_point, 2, (0,255,0), -1)
	except AttributeError:
		print('No se ha detectado aruco ID=4')
		pass
	cv2.imshow("camera_route",callback_imagen.image_np)
	cv2.waitKey(1)
	
def position_control():
	rospy.init_node('position_control', anonymous=True)
	rospy.Subscriber('/coord_waypoints_update_indicator', String, callback_waypoints)#viene de path_planning
	rospy.Subscriber("/coord_obstaculos_aruco", String, callback_angulos)#viene de mapa2d, mapa2d es el publisher
	rospy.Subscriber('/imagen_procesada', CompressedImage, callback_imagen)
	print("######### first waypoint update #########")
	with open('/home/proyinv/ponce_ws/src/path_planning/scripts/route_nodes.json') as f:
		data_json = json.load(f)
	n=0
	waypoints=[]
	global waypoints_list
	for item in data_json['waypoints4']:
		index=item['index']
		coord=item['coord']
		waypoints.append(coord)
		n=n+1
	waypoints_list=list(waypoints)
	callback_waypoints.end=False#False cuando aun no se ha terminado con los waypoints
	rospy.spin()	

if __name__ == '__main__':
	position_control()
