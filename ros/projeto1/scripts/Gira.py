import rospy

import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def gira(velocidade_saida):
	ang_speed = 0.4
	vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
	velocidade_saida.publish(vel)
