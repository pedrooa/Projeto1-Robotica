#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import rospy
import numpy as np
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import Header
import deteccao
from Fugir import foge
from Seguir import segue
from Survive import sobrevive
import Gira
import cor
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

atraso = 0.4E9
check_delay = True # configure as needed
madfox_tamanho = 0
objeto_tamanho = 0
achou_obstaculo = False
velocidadenula = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


# configuracao da imagem detectada
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDITREE = 0
flannParam = dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann = cv2.FlannBasedMatcher(flannParam,{})

img1 = cv2.imread("/home/borg/catkin_ws/src/Projeto1-Robotica/ros/projeto1/scripts/madfox.jpg",0)
time.sleep(2)
trainKP,trainDesc = sift.detectAndCompute(img1,None)


def recebe_imagem(imagem):
	global cv_image
	global media_madfox
	global centro_madfox
	global achou_madfox
	global madfox_tamanho
	global media_objeto
	global centro_objeto
	global achou_objeto
	global objeto_tamanho
	global achou_obstaculo


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay:
		return 
	try:
		# antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media_madfox, centro_madfox, achou_madfox, madfox_tamanho = deteccao.detecta_imagem(cv_image)
		media_objeto, centro_objeto, objeto_tamanho = cor.identifica_cor(cv_image)
		#media, centro, area = cormodule.identifica_cor(cv_image)
		# depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e);


def recebe_laser(dado):
	global Distancias
	global achou_obstaculo
	print("chegoulaser")
	Distancias = np.array(dado.ranges).round(decimals=2)
	for distancia in Distancias[300:]:
		if distancia < 0.3 and distancia > 0.1:
			print(distancia)
			achou_obstaculo = True
			print(achou_obstaculo)
			print("achou obstaculo")
			rospy.sleep(0.01)
		else:
			print("nao_obstaculo")
			achou_obstaculo = False
			rospy.sleep(0.01)



class Rest(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto', 'AchouObstaculo','NaoAchou'])

	def execute(self, userdata):
		velocidade_saida.publish(velocidadenula)
		if achou_obstaculo == True:
			print("achouobstaculo")
			sobrevive(Distancias, velocidade_saida)
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			#Gira.gira(velocidade_saida)
			return 'NaoAchou'

class Sobrevive(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou'])

	def execute(self, userdata):
		if achou_obstaculo == True:
			velocidade_saida.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -3)))
			#sobrevive(Distancias, velocidade_saida)
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			return 'NaoAchou'


class Seguir(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou'])

	def execute(self, userdata):
		if achou_obstaculo == True:
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			segue(velocidade_saida, media_objeto, centro_objeto)
			return 'AchouObjeto'
		else:
			return 'NaoAchou'

class Fugir(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou'])

	def execute(self, userdata):
		if achou_obstaculo == True:
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			foge(velocidade_saida, media_madfox, centro_madfox, achou_madfox)
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			return 'NaoAchou'

def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, recebe_laser)
	recebedor_imagem = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, recebe_imagem, queue_size=10, buff_size = 2**24)
	

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
		# Add states to the container
		#smach.StateMachine.add('LONGE', Longe(), 
		#                       transitions={'ainda_longe':'ANDANDO', 
		#                                    'perto':'terminei'})
		#smach.StateMachine.add('ANDANDO', Andando(), 
		#                       transitions={'ainda_longe':'LONGE'})
		
		smach.StateMachine.add('REST', Rest(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo': 'SOBREVIVE'})
		smach.StateMachine.add('SOBREVIVE', Sobrevive(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo':'SOBREVIVE'})
		smach.StateMachine.add('SEGUIR', Seguir(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo':'SOBREVIVE'})
		smach.StateMachine.add('FUGIR', Fugir(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo':'SOBREVIVE'})

	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()

if __name__ == '__main__':
	print("Main")
	main()

