#!/usr/bin/env python
import rospy
import json
import numpy as np
import time
import math
from collisions_monitor.msg import colision_msg
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from shapely.geometry import LineString,Point
import shapely
import cv2

import itertools
import os
from math import sqrt,cos,sin,atan2
from math import modf

pub_colision = rospy.Publisher('/colisiones', colision_msg, queue_size=10)
pub_image = rospy.Publisher('/imagen_procesada_colision', CompressedImage, queue_size=10)
pub_waypoints = rospy.Publisher('/coord_waypoints_update_indicator', String,queue_size=10)

obstaculos=[]
obstaculos_global=[]
aruco=[]
aruco_global=[]
aruco_aux=[]
ids=[]
ids_global=[]
ids_aux=[]
angulos=[]
angulos_global=[]
center_list=[]
waypoints_list4=[]
waypoints_list3=[]
waypoints_list2=[]
waypoints_list1=[]
all_waypoints_list=[]

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
	
def aruco_center_calc(monitored_aruco_corners):
	center = [(monitored_aruco_corners[0]+monitored_aruco_corners[4])/2, (monitored_aruco_corners[1]+monitored_aruco_corners[5])/2]
	return center
	
def callback_waypoints(data): #Aqui se hara el pre-analisis de rutas 
	global all_waypoints_list

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
	
	#lista de listas donde estan los segmentos de cada ruta por separado
	segments_list=[[],[],[],[]] #Se definen 4 espacios para 4 robots
	for each_list in all_waypoints_list:
		if each_list==[0] or len(each_list[0])==1:
				pass
		else:
			for n in range(len(each_list[0])):

				if n <= (len(each_list[0]))-2 :
					line = LineString([(each_list[0][n][0], each_list[0][n][1]), (each_list[0][n+1][0], each_list[0][n+1][1])])
					#other = LineString([(each_list[0][n+1][0]+1, each_list[0][n+1][1]+1), (each_list[0][n+2][0], each_list[0][n+2][1])])
					segments_list[all_waypoints_list.index(each_list)-1].append(line)
					#segments_list.append(other)


				else:
					pass				
#	
#	
#	for each_route in segments_list:
#		print('route')
#		for each_line in each_route:
#			print(each_line)

	#necesito una lista que incluya cada linea/segmento de cada ruta.
	all_segments_list=[]
	for each_list in segments_list:
		for each_line in each_list:
			all_segments_list.append(each_line)
	
	#print(all_segments_list)
	
			
	intersections_list=[]
	for a, b in itertools.combinations(all_segments_list, 2):
#		print('intersect?')
		int_pt = a.intersection(b)
		if int_pt:
			point_of_intersection = [int_pt.x, int_pt.y]
			#print(point_of_intersection)
			
			x_dec,x_int = modf(point_of_intersection[0])
			y_dec,y_int = modf(point_of_intersection[1])
			if y_dec != 0 or x_dec != 0: #Se identifican los puntos q tienen decimal porq los q corresponden a waypoints siempre son enteros
				intersections_list.append(point_of_intersection)
		else:
		#	print('vacio')
			pass
	print(intersections_list)
	
	where_collision=[]
	route_to_collision=[[],[],[],[]]
	for each_point in intersections_list:
		for each_list in segments_list: #segments list=[[],[],[],[]]
			for each_segment in each_list:
				point_shapely=Point(each_point[0],each_point[1])
				if each_segment.distance(point_shapely) < 1e-8:
					where_collision.append(segments_list.index(each_list)+1)
					segment_number=each_list.index(each_segment)
					#print(segment_number)
#					if segment_number==0:
#						route_to_collision[segments_list.index(each_list)].append(each_segment)
#					else:
					for i in range(segment_number+1):
						route_to_collision[segments_list.index(each_list)].append(each_list[i])
					
				else:
					pass
	print(where_collision)
	print("#############")
	route_to_collision_final = [[],[],[],[]] #REMOVER LOS REPETIDOS EN CADA LISTA DE route_to_collision
	for each_list in route_to_collision:
		for each_route in each_list:
		    if  each_route not in route_to_collision_final[route_to_collision.index(each_list)]:
			route_to_collision_final[route_to_collision.index(each_list)].append(each_route)
		
#	for each_route in route_to_collision_final:
#		print('route')
#		for each_line in each_route:
#			print(each_line)
		
	#print(route_to_collision_final[0][0].coords[0])
	print('#### Nodes to clision process')
	nodes_to_collision=[[],[],[],[]]
	for each_list in route_to_collision_final:
		for each_line in each_list:
	
			nodes_to_collision[route_to_collision_final.index(each_list)].append([each_line.coords[0][0],each_line.coords[0][1]])
			distancias = []
			for each_point in intersections_list:

				point_shapely=Point(each_point[0],each_point[1]) #punto de cruze
				coord_shapely=Point(each_line.coords[0][0],each_line.coords[0][1]) #coord del punto q se acaba de append
				#sacar las distancias entre cada pnto de cruze y el coord_shapely q es el origen de cada linea
				if each_line.distance(point_shapely) < 1e-8: #si es q el punto esta en el segmento
					distancias.append(point_shapely.distance(coord_shapely))
				else:
					distancias.append(10000)
			#una vez acabados los puntos de interseccion
			intersections_list_np = np.array(intersections_list)
			distancias_np = np.array(distancias)
			inds = distancias_np.argsort()
			sortedPoints = intersections_list_np[inds]
#			print(sortedPoints)
			
			for each_point in sortedPoints:
				point_shapely=Point(each_point[0],each_point[1])
				if each_line.distance(point_shapely) < 1e-8: #si es q el punto esta en el segmento
					nodes_to_collision[route_to_collision_final.index(each_list)].append([round(each_point[0]),round(each_point[1])])				
			
	print('nodes to colision',nodes_to_collision)
	
	#La idea es que se va a calcular el tiempo q le toma a cada robot llegar al punto de colision
	# esto sirve para determinar un tiempo prudencial en el q se retrasa al robot con menor prioridad
	# hasta que se sepa/estime que el otro ya paso por el punto de colision
	

	
def callback(data):
	#os.system('clear')
	#print("######### COLLISION MONITOR #########")
	
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
#	print("Aruco IDs:")
#	print(ids)
	n=0
	angulos=[]
	for item in data_json['angulos']:
		index=item['index']
		aruco_angulo=item['angulo']
		angulos.append(aruco_angulo)
		n=n+1
	angulos_global=angulos
#	print("Aruco angulos:")
#	print(angulos)
	
	##########3 MONITOR de DISTANCIAS ############3
	center_list=[]
	for each_aruco in aruco:
		aruco_center=aruco_center_calc(each_aruco)
		center_list.append(aruco_center)
	#print(center_list)
	
	
	ids_to_stop_list=[]
	for a, b in itertools.combinations(center_list, 2):
		
		index_a=center_list.index(a)
		index_b=center_list.index(b)

#		
		controlled_index_a =(ids[index_a])
		controlled_index_b =(ids[index_b])
		
		angulo_actual_a=read_angle()[index_a]
		angulo_actual_b=read_angle()[index_b]
		
		controlled_aruco_corners_a=read_aruco()[index_a]
		controlled_aruco_corners_b=read_aruco()[index_b]
		
		aruco_center_a=aruco_center_calc(controlled_aruco_corners_a) #centro del aruco controlado, [x,y]
		aruco_center_b=aruco_center_calc(controlled_aruco_corners_b) #centro del aruco controlado, [x,y]
		
		aruco_centerpoint_a=[(controlled_aruco_corners_a[0]+controlled_aruco_corners_a[2])/2, (controlled_aruco_corners_a[1]+controlled_aruco_corners_a[3])/2] # el centerpoint representaria la bolaloca
		aruco_centerpoint_b=[(controlled_aruco_corners_b[0]+controlled_aruco_corners_b[2])/2, (controlled_aruco_corners_b[1]+controlled_aruco_corners_b[3])/2] # el centerpoint representaria la bolaloca
		
		aruco_centerback_a=[(controlled_aruco_corners_a[6]+controlled_aruco_corners_a[4])/2, (controlled_aruco_corners_a[7]+controlled_aruco_corners_a[5])/2] # centerback el punto medio del eje de ruedas
		aruco_centerback_b=[(controlled_aruco_corners_b[6]+controlled_aruco_corners_b[4])/2, (controlled_aruco_corners_b[7]+controlled_aruco_corners_b[5])/2] # centerback el punto medio del eje de ruedas
		
		a=40; #distancia q define el punto de origen del circulo 
		phi=math.radians(angulo_actual_a)
		xr_a=aruco_centerpoint_a[0]+a*cos(phi);
		yr_a=aruco_centerpoint_a[1]+a*sin(-phi);
		a_point=(int(xr_a), int(yr_a))
		
		b=40;
		phi=math.radians(angulo_actual_b)
		xr_b=aruco_centerpoint_b[0]+b*cos(phi);
		yr_b=aruco_centerpoint_b[1]+b*sin(-phi);
		b_point=(int(xr_b), int(yr_b))
		
		# Driver code #RADIO A Y B
		ra = 50
		rb = 50
		
		id_to_stop=max(ids[index_a][0],ids[index_b][0])
		
		t = circle(a_point, b_point, ra, rb) 
		if (t >= 0):
			#print("Circle touch each other.") 
			#cmd_pub.cmd_stop=True #True si necesita detenerse
			#cmd_pub.aruco_id=id_to_stop #Se publica siempre el Aruco conid mayor que es el que debe detenerse
			#pub_colision.publish(cmd_pub)
			ids_to_stop_list.append(id_to_stop)
			print('cmd detener robot a',id_to_stop)
			
		else :
			#print("Circle do not touch each other.") 
			ids_to_stop_list.append(0)

			
			
	#HACER PUBLISH DESPUES DE CADA FOR, con todos los ids q deben detenerse
	cmd_pub=colision_msg()
	cmd_pub.aruco_id=ids_to_stop_list
	pub_colision.publish(cmd_pub)
	#print(ids_to_stop_list)


	

def callback_imagen(data):

	br = CvBridge()
	np_arr = np.fromstring(data.data, np.uint8)
	callback_imagen.image_np = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR)
	
	for i in range(4):
		try:
			controlled_index = read_ids().index([i+1])
			angulo_actual=read_angle()[controlled_index]
			controlled_aruco_corners=read_aruco()[controlled_index]
			aruco_center=aruco_center_calc(controlled_aruco_corners) #centro del aruco controlado, [x,y]
			aruco_centerpoint=[(controlled_aruco_corners[0]+controlled_aruco_corners[2])/2, (controlled_aruco_corners[1]+controlled_aruco_corners[3])/2] # el centerpoint representaria la bolaloca
			aruco_centerback=[(controlled_aruco_corners[6]+controlled_aruco_corners[4])/2, (controlled_aruco_corners[7]+controlled_aruco_corners[5])/2] # centerback el punto medio del eje de ruedas
	
			a=40;
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
			cv2.circle(callback_imagen.image_np,a_point, 50, (0,255,0), 2) ## RADIO MOD is 2 argument
			
			
		except ValueError:
			#print('No se ha detectado aruco ID_ IMAGEN PROCESS',i+1)
			pass

	img_pub = CompressedImage()
	img_pub.header.stamp = rospy.Time.now()
	img_pub.format = "jpeg"
	img_pub.data = np.array(cv2.imencode('.jpg', callback_imagen.image_np)[1]).tostring()
	pub_image.publish(img_pub)
	#cv2.imshow("collision check",callback_imagen.image_np)
	#cv2.waitKey(1)

def circle((x1,y1),(x2,y2), r1, r2):
   
    distSq = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2); 
    radSumSq = (r1 + r2) * (r1 + r2); 
    if (distSq <= radSumSq):# and (distSq > radSumSq/2):
        return 1 
    else:
        return -1 


def colision_monitor():

	rospy.init_node('colision_monitor', anonymous=True)
	rospy.Subscriber("/coord_obstaculos_aruco", String, callback)#viene de mapa2d, 
	rospy.Subscriber('/coord_waypoints_update_indicator', String, callback_waypoints)#viene de path_planning
	rospy.Subscriber('/imagen_procesada', CompressedImage, callback_imagen)
	rospy.spin()

if __name__ == '__main__':
	colision_monitor()
