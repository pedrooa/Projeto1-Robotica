#! /usr/bin/env python
# -*- coding:utf-8 -*-

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
media = []
centro = []
atraso = 1.5
achou = 0

check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados


# configuracao da imagem detectada
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDITREE = 0
flannParam = dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann = cv2.FlannBasedMatcher(flannParam,{})

img1 = cv2.imread("/home/borg/catkin_ws/src/Projeto1-Robotica/ros/projeto1/scripts/madfox.jpg",0)
time.sleep(6)
trainKP,trainDesc = sift.detectAndCompute(img1,None)




def detecta_imagem(frame):
	achou = 0

######## algoritmo de deteccao da imagem (copiada do projeto madfox)

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray,(5,5),0)
	bordas_color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
	queryKP,queryDesc = sift.detectAndCompute(gray,None)
	matches = flann.knnMatch(queryDesc,trainDesc,k=2)
	goodMatch = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
			goodMatch.append(m)
	MIN_MATCH_COUNT = 30
	if len(goodMatch) > MIN_MATCH_COUNT:
		achou = 1

		tp = []
		qp = []
		for m in goodMatch:
			tp.append(trainKP[m.trainIdx].pt)
			qp.append(queryKP[m.queryIdx].pt)
		tp,qp=np.float32((tp,qp))
		H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
		h,w = img1.shape
		trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
		queryBorder=cv2.perspectiveTransform(trainBorder,H)


		x0 = queryBorder[0][0][0]
		x1 = (queryBorder[0][1][0])
		x2 = (queryBorder[0][2][0])
		x3 = (queryBorder[0][3][0])
		y0 = (queryBorder[0][0][1])
		y1 = (queryBorder[0][1][1])
		y2 = (queryBorder[0][2][1])
		y3 = (queryBorder[0][3][1])
		media_x = (x0+x1+x2+x3)/4.0
		media_y = (y0+y1+y2+y3)/4.0

		media = (media_x, media_y)   # probelma no y centro centro da imagem detectada

		centro = (frame.shape[1]//2, frame.shape[0]//2)  # centro da webcam

		dif_x = media[0]-centro[0]
		dif_y = media[1]-centro[1]

		##tamanho da imagem
		tx = x3 - x0
		ty = y1 - y0
		tamanho = (tx*ty)


		cv2.polylines(bordas_color,[np.int32(queryBorder)],True,(0,255,0),5)
	else:
		media = (0,0)
		print "Raposa nao encontrada- %d/%d"%(len(goodMatch),MIN_MATCH_COUNT)

	print('funcionando')
	centro = (frame.shape[0]//2, frame.shape[1]//2)
	return media, centro, achou


def roda_todo_frame(imagem):
	print("teste")
	global cv_image
	global achou
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, achou = detecta_imagem(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":
	print('funcionando1')

	rospy.init_node("projeto")
	print('funcionando2')
	# Para usar a Raspberry Pi
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	print('funcionando3')
	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	print('funcionando4')

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

			if len(media) != 0 and len(centro) != 0 and achou == 1:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if dif_x < 0: # Vira a esquerda
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-5))
					time.sleep(0.7)
					print('esquerda')
				if dif_x > 0: # Vira a direita
					vel = Twist(Vector3(0,0,0), Vector3(0,0,5))
					time.sleep(0.7)
			else: 
				print('nao achou')
				vel = Twist(Vector3(0.01,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")