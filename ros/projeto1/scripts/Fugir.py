import rospy
import time
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def foge(velocidade_saida, media_madfox, centro_madfox, achou_madfox):
	vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

	if len(media_madfox) != 0 and len(centro_madfox) != 0 and achou_madfox == 1:
		dif_x = media_madfox[0]-centro_madfox[0]
		dif_y = media_madfox[1]-centro_madfox[1]
	if dif_x < 0: # Vira a esquerda
		vel = Twist(Vector3(0,0,0), Vector3(0,0,-5))
		time.sleep(0.1)
		print('esquerda')
	if dif_x > 0: # Viraa direita
			vel = Twist(Vector3(0,0,0), Vector3(0,0,5))
			time.sleep(0.1)
	velocidade_saida.publish(vel)
	rospy.sleep(0.01)