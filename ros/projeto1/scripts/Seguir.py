import rospy
import time
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def segue(velocidade_saida, media_objeto, centro_objeto, tamanho_objeto):  #funcao para o robo seguir o objeto (quanto mais longe o objeto mais rapido o robo vai)
	vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
	print(tamanho_objeto)
	if len(media_objeto) != 0 and len(centro_objeto) != 0:
		dif_x = media_objeto[0] - centro_objeto[0]
		dif_y = media_objeto[1] - centro_objeto[1]
		if math.fabs(dif_x)<50: # Se a media_objeto estiver muito proxima do centro_objeto anda para frente
			if tamanho_objeto < 5000:
				vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
			elif tamanho_objeto < 30000:
				vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			else tamanho_objeto > 70000:
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))

		elif dif_x > 150:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
		elif dif_x < -150:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
		else:
			if dif_x > 0: # Vira a direita
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
			else: # Vira a esquerda
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
		velocidade_saida.publish(vel)
		rospy.sleep(0.01)