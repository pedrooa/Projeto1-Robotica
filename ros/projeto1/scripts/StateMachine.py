#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import rospy
import numpy
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import Survive
import deteccao
import Fugir
import Seguir
import Gira
import cor

atraso_maximo = 1.5
check_delay = False # configure as needed

# configuracao da imagem detectada
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDITREE = 0
flannParam = dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann = cv2.FlannBasedMatcher(flannParam,{})

img1 = cv2.imread("/home/borg/catkin_ws/src/Projeto1-Robotica/ros/projeto1/scripts/madfox.jpg",0)
time.sleep(4)
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


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso_maximo and check_delay:
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
	Distancias = np.array(dado.ranges).round(decimals=2)
	for distancia in Distancias[300:]:
		if distancia < 0.5 and distancia != 0.0:
			achou_obstaculo == True
		else:
			achou_obstaculo == False


class Rest(smach.State):
	def __init__(self):
        smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto', 'NaoAchou'])

    def execute(self, userdata):
  		if madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			Gira.gira()
			return 'NaoAchou'

class Sobrevive(smach.State):
	def __init__(self):
        smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou'])

    def execute(self, userdata):
    	if achou_obstaculo:
    		Survive.sobrevive()
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
		if achou_obstaculo:
    		return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			Seguir.segue()
			return 'AchouObjeto'
		else:
			return 'NaoAchou'

class Fugir(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou'])

	def execute(self, userdata):
		if achou_obstaculo:
    		return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			Fugir.foge()
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
	recebedor_imagem = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, recebe_imagem, queue_size=10, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, recebe_laser)

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
	                            'NaoAchou': 'REST'})
	    smach.StateMachine.add('SOBREVIVE', Sobrevive(),
	                            transitions={'AchouMadfox': 'FUGIR',
	                            'AchouObjeto': 'SEGUIR',
	                            'NaoAchou': 'REST'}
	                            'AchouObstaculo':'SOBREVIVE')
	    smach.StateMachine.add('SEGUIR', Seguir(),
	                            transitions={'AchouMadfox': 'FUGIR',
	                            'AchouObjeto': 'SEGUIR',
	                            'NaoAchou': 'REST'}
	                            'AchouObstaculo':'SOBREVIVE')
	    smach.StateMachine.add('FUGIR', Fugir(),
	                            transitions={'AchouMadfox': 'FUGIR',
	                            'AchouObjeto': 'SEGUIR',
	                            'NaoAchou': 'REST'}
	                            'AchouObstaculo':'SOBREVIVE')

	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()

