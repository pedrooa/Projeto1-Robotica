#! /usr/bin/env python
# -*- coding:utf-8 -*-
#delta = 0.12s
import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math


global t
t = 3
global l 
l = np.zeros(t)
global prox 
prox = 0
global tempo

global colisao
colisao = False


def leu_imu(dado):
	global prox
	global t
	global tempo
	global colisao

	tempo2 = dado.header.stamp
	delta = rospy.Duration(secs=0.12)
	l[prox%t] = dado.linear_acceleration.x 

	media = np.mean(l)
	print(media)

	if colisao == False:
		if media < -0.8:
			print('bateus')
			tempo = dado.header.stamp
			colisao = True

	if colisao == True :
		if (tempo2-tempo) < 30*delta :
			velocidade = Twist(Vector3(-0.05, -0.05, -0.05), Vector3(-0, -0, -0.2))
			velocidade_saida.publish(velocidade)
		else:
			colisao = False

	print(colisao)


	prox+=1


	quat = dado.orientation
	print(quat)
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	mensagem = """
	Tempo: {:}
	Orientação: {:.2f}, {:.2f}, {:.2f}
	Vel. angular: x {:.2f}, y {:.2f}, z {:.2f}\
	Aceleração linear:
	x: {:.2f}
	y: {:.2f}
	z: {:.2f}
""".format(dado.header.stamp, angulos[0], angulos[1], angulos[2], dado.angular_velocity.x, dado.angular_velocity.y, dado.angular_velocity.z, dado.linear_acceleration.x, dado.linear_acceleration.y, dado.linear_acceleration.z)
	print(mensagem)

	


if __name__=="__main__":

	rospy.init_node("le_imu")
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	
	recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu)

	while not rospy.is_shutdown():
		print("Main loop")
		rospy.sleep(2)
		
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -3))

