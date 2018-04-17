#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

global x  
x = True

def scaneou(dado):
	global x
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	Distancias = np.array(dado.ranges).round(decimals=2)
	#print(Distancias[315])
	for distancia in Distancias[355:]:

		if distancia < 0.4 and distancia != 0.0:
			x = False

			velocidade = Twist(Vector3(-0.2, -0.2, -0.2), Vector3(0, 0, -3))

		else:
			x = True
			velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))
		
		velocidade_saida.publish(velocidade)

	for distancia in Distancias[:5]:

		if distancia < 0.4 and distancia != 0.0:
			x = False

			velocidade = Twist(Vector3(-0.2, -0.2, -0.2), Vector3(0, 0, -3))

		else:
			x = True
			velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))
		
		velocidade_saida.publish(velocidade)


	print(x)

	if x == True:

		for distancia in Distancias[300:350]:

			if distancia < 0.3 and distancia != 0.0:
				
				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 3))
	#		else: 
	#			velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))

			velocidade_saida.publish(velocidade)


		for distancia in Distancias[10:60]:

			if distancia < 0.3 and distancia != 0.0:
				#print(distancia)
				

				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -3))
		#else: 
		#	velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))

			velocidade_saida.publish(velocidade)


def sobrevive(Distancias, velocidade_saida):
	global x
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	#Distancias = np.array(dado.ranges).round(decimals=2)
	#print(Distancias[315])
	for distancia in Distancias[355:]:

		if distancia < 0.4 and distancia != 0.0:
			x = False

			velocidade = Twist(Vector3(-0.2, -0.2, -0.2), Vector3(0, 0, -3))

		else:
			x = True
			velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))
		
		velocidade_saida.publish(velocidade)

	for distancia in Distancias[:5]:

		if distancia < 0.4 and distancia != 0.0:
			x = False

			velocidade = Twist(Vector3(-0.2, -0.2, -0.2), Vector3(0, 0, -3))

		else:
			x = True
			velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))
		
		velocidade_saida.publish(velocidade)


	print(x)

	if x == True:

		for distancia in Distancias[300:350]:

			if distancia < 0.3 and distancia != 0.0:
				
				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 3))
	#		else: 
	#			velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))

			velocidade_saida.publish(velocidade)


		for distancia in Distancias[10:60]:

			if distancia < 0.3 and distancia != 0.0:
				#print(distancia)
				

				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -3))
		#else: 
		#	velocidade = Twist(Vector3(0.2, 0.2, 0.2), Vector3(0, 0, 0))

			velocidade_saida.publish(velocidade)



if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():
		print("Oeee")
		
		rospy.sleep(2)






