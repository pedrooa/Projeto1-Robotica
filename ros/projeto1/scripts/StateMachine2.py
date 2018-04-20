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
from sensor_msgs.msg import Image, CompressedImage, LaserScan, Imu
from std_msgs.msg import Header
import deteccao
from Fugir import foge
from Seguir import segue
from Survive import sobrevive
import colisao_imu
import Gira
import cor
import cv2
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()

atraso = 0.4E9
check_delay = True 
madfox_tamanho = 0
objeto_tamanho = 0
achou_obstaculo = False
velocidadenula = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
global t
t = 3
global l 
l = np.zeros(t)
global prox 
prox = 0
global tempo

global colisao
colisao = False
global x
x=0
tempo2 = rospy.Duration(secs=0)
tempo = rospy.Duration(secs=0)

global fundo
global foto

# configuracao da imagem detectada
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDITREE = 0
flannParam = dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann = cv2.FlannBasedMatcher(flannParam,{})



img1 = cv2.imread("/home/borg/catkin_ws/src/Projeto1-Robotica/ros/projeto1/scripts/madfox.jpg",0)
time.sleep(2)
trainKP,trainDesc = sift.detectAndCompute(img1,None)




def sub(frame): #funcao para tirar foto do fundo
	fundo = frame.copy()
	fundo = cv2.cvtColor(fundo, cv2.COLOR_BGR2GRAY)
	return fundo


def sub2(frame,fundo):  #funcao para tirar foto do objeto e realizar a subtracao de fundo, ja retornando os valores para o sift deterctar a nova imagem
	foto1 = frame.copy()
	foto = cv2.cvtColor(foto1, cv2.COLOR_BGR2GRAY)
	diferenca = cv2.subtract(foto,fundo)
	diferenca2 = cv2.subtract(fundo,foto)
	or_img = cv2.bitwise_or(diferenca, diferenca2)
	ret,limiar = cv2.threshold(or_img,np.percentile(or_img, 97),255,cv2.THRESH_BINARY)
	kernel = np.ones((4,4))
	limiar_close = cv2.morphologyEx(limiar, cv2.MORPH_CLOSE, kernel)
	segmentado_cor = cv2.morphologyEx(limiar_close,cv2.MORPH_CLOSE,np.ones((7, 7)))
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
	maior_contorno = None
	objeto_tamanho = 0

	for cnt in contornos:
		area = cv2.contourArea(cnt)
		if area > objeto_tamanho:
			maior_contorno = cnt
			objeto_tamanho = area
	x, y, w, h = cv2.boundingRect(maior_contorno)
	objeto = foto1[y:(y+h),x:(x+w),:]
	cv2.imwrite("objeto.jpg", objeto)

	sift = cv2.xfeatures2d.SIFT_create()
	FLANN_INDEX_KDITREE = 0
	flannParam = dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
	flann = cv2.FlannBasedMatcher(flannParam,{})
	trainKP,trainDesc = sift.detectAndCompute(objeto,None)
	rospy.sleep(2)

	return trainKP, trainDesc, objeto 


def recebe_imagem(imagem):  #funcao para receber imagem do robo e usar as fucnoes de identificao de imagem
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
	global trainKP
	global trainDesc


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay:
		return 
	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media_madfox, centro_madfox, achou_madfox, madfox_tamanho = deteccao.detecta_imagem(cv_image,trainKP,trainDesc,img1)
		media_objeto, centro_objeto, objeto_tamanho = cor.identifica_cor(cv_image)
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e);


def recebe_laser(dado):  #funcao para receber laser do robo
	global Distancias
	global achou_obstaculo
	Distancias = np.array(dado.ranges).round(decimals=2)
	for distancia in Distancias[0:60]:
		if distancia < 0.3 and distancia > 0.1:
			achou_obstaculo = True
			break
		else:
			achou_obstaculo = False
	for distancia in Distancias[300:]:
		if distancia < 0.3 and distancia > 0.1:
			achou_obstaculo = True
			break
		else:
			achou_obstaculo = False


def leu_imu(dado):  #funcao para receber informacoes da IMU do robo
	global prox
	global t
	global tempo
	global colisao
	global x
	global tempo2
	
	l[prox%t] = dado.linear_acceleration.x 
	tempo2 = dado.header.stamp
	media = np.mean(l)
	if colisao == False:
		if media < -3.0:
			tempo = dado.header.stamp
			colisao = True
		else:
			colisao = False
	prox +=1


class Tirafoto(smach.State):  #estado incial para aprender objeto novo
	def __init__(self):
		smach.State.__init__(self, outcomes=['NaoAchou'])

	def execute(self, userdata):
		global trainDesc
		global trainKP
		rospy.sleep(3)
		fundo = sub(cv_image)
		print('tirou')
		rospy.sleep(6)
		print('prepara')
		rospy.sleep(2)
		trainKP,trainDesc, img1 = sub2(cv_image,fundo)

		return 'NaoAchou'

class Rest(smach.State):  #estado de rest(nao segue nem desvia de nada)
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto', 'AchouObstaculo','NaoAchou', 'Colidiu'])

	def execute(self, userdata):
		velocidade_saida.publish(velocidadenula)
		rospy.sleep(0.01)

		if colisao == True:
			return 'Colidiu'
		if achou_obstaculo == True:
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			return 'NaoAchou'

class Sobrevive(smach.State):  #estado para sobreviver(desviar de objetos)
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou', 'Colidiu'])

	def execute(self, userdata):
		if colisao == True:
			return 'Colidiu'
		elif achou_obstaculo == True:
			sobrevive(Distancias, velocidade_saida)
			
			#velocidade_saida.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -3)))
			rospy.sleep(0.1)
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			return 'NaoAchou'


class Seguir(smach.State): () #estado para seguir o objeto de cor especifica
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou', 'Colidiu'])

	def execute(self, userdata):
		if colisao == True:
			return 'Colidiu'
		elif achou_obstaculo == True:
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			segue(velocidade_saida, media_objeto, centro_objeto, objeto_tamanho)
			return 'AchouObjeto'
		else:
			return 'NaoAchou'

class Fugir(smach.State):  #estado para fugir do objeto de imagem especifica
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou', 'Colidiu'])

	def execute(self, userdata):
		if colisao == True:
			return 'Colidiu'
		elif achou_obstaculo == True:
			return 'AchouObstaculo'
		elif madfox_tamanho > objeto_tamanho:
			foge(velocidade_saida, media_madfox, centro_madfox, achou_madfox)
			return 'AchouMadfox'
		elif objeto_tamanho > madfox_tamanho:
			return 'AchouObjeto'
		else:
			return 'NaoAchou'

class Colisao(smach.State):  #funcao para reacao a colisao do robo
	def __init__(self):
		smach.State.__init__(self, outcomes=['AchouMadfox', 'AchouObjeto','AchouObstaculo','NaoAchou', 'Colidiu'])

	def execute(self, userdata):
		global colisao
		delta = rospy.Duration(secs=0.12)
		if (tempo2-tempo) < 10*delta :
			velocidade = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.01)
		else:
			velocidade = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.01)
			colisao = False
		if colisao == True:
			return 'Colidiu'			
		elif achou_obstaculo == True:
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
	recebe_scan = rospy.Subscriber("/scan", LaserScan, recebe_laser)
	recebedor_imagem = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, recebe_imagem, queue_size=10, buff_size = 2**24)
	recebe_imu = rospy.Subscriber("/imu", Imu, leu_imu)
	

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	sm = smach.StateMachine(outcomes=['terminei'])

	with sm:
		smach.StateMachine.add('TIRANDOFOTO', Tirafoto(),
								transitions={'NaoAchou': 'REST',})
		
		smach.StateMachine.add('COLISAO', Colisao(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo': 'SOBREVIVE',
								'Colidiu': 'COLISAO'})
		smach.StateMachine.add('REST', Rest(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo': 'SOBREVIVE',
								'Colidiu': 'COLISAO'})
		smach.StateMachine.add('SOBREVIVE', Sobrevive(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo': 'SOBREVIVE',
								'Colidiu': 'COLISAO'})
		smach.StateMachine.add('SEGUIR', Seguir(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo': 'SOBREVIVE',
								'Colidiu': 'COLISAO'})
		smach.StateMachine.add('FUGIR', Fugir(),
								transitions={'AchouMadfox': 'FUGIR',
								'AchouObjeto': 'SEGUIR',
								'NaoAchou': 'REST',
								'AchouObstaculo': 'SOBREVIVE',
								'Colidiu': 'COLISAO'})

	outcome = sm.execute()

if __name__ == '__main__':
	print("Main")
	main()

