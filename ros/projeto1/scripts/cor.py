#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cv_image = None
media_objeto = []
centro_objeto = []
atraso = 0.4E9

check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def identifica_cor(frame):  #funcao para identificar objeto da cor especificada usada no robo

	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	cor_menor = np.array([40, 90, 115])
	cor_maior = np.array([70, 135, 186])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
	# que um quadrado 7x7. É muito útil para juntar vários 
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

	maior_contorno = None
	objeto_tamanho = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > objeto_tamanho:
	        maior_contorno = cnt
	        objeto_tamanho = area

	# Encontramos o centro_objeto do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None :
	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
	    media_objeto = maior_contorno.mean(axis=0)
	    media_objeto = media_objeto.astype(np.int32)
	    cv2.circle(frame, tuple(media_objeto), 5, [0, 255, 0])
	else:
	    media_objeto = (0, 0)
	    objeto_tamanho = 0

	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro_objeto = (frame.shape[0]//2, frame.shape[1]//2)

	return media_objeto, centro_objeto, objeto_tamanho



def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media_objeto
	global centro_objeto

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media_objeto, centro_objeto, objeto_tamanho = identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	


if __name__=="__main__":

	rospy.init_node("cor")
	# Para usar a Raspberry Pi
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	
	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media_objeto) != 0 and len(centro_objeto) != 0:
				dif_x = media_objeto[0]-centro_objeto[0]
				dif_y = media_objeto[1]-centro_objeto[1]
				if math.fabs(dif_x)<30: # Se a media_objeto estiver muito proxima do centro_objeto anda para frente
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				else:
					if dif_x > 0: # Vira a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					else: # Vira a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


