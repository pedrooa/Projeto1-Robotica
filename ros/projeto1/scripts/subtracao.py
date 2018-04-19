	#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import cv2
from matplotlib import pyplot as plt
import time as t
import rospy




def sub(frame):
	fundo = frame.copy()
		
	fundo = cv2.cvtColor(fundo, cv2.COLOR_BGR2GRAY)
	#fundo = cv2.cvtColor(fundo, cv2.COLOR_BGR2RGB)
	cv2.imwrite("fundo.jpg", fundo)
	print('troq+ue')
	return fundo


def sub2(frame):
	#ret, frame = cap.read()
	print('snap')
	foto1 = frame.copy()
	foto = cv2.cvtColor(foto1, cv2.COLOR_BGR2GRAY)
	#foto = cv2.cvtColor(foto, cv2.COLOR_BGR2RGB)
	cv2.imwrite("foto.jpg", foto)
	diferenca = cv2.subtract(foto,fundo)
	diferenca2 = cv2.subtract(fundo,foto)
	or_img = cv2.bitwise_or(diferenca, diferenca2)
	ret,limiar = cv2.threshold(or_img,np.percentile(or_img, 97),255,cv2.THRESH_BINARY)
	kernel = np.ones((4,4))
	limiar_open = cv2.morphologyEx(limiar, cv2.MORPH_OPEN, kernel)
	limiar_close = cv2.morphologyEx(limiar, cv2.MORPH_CLOSE, kernel)
	cv2.imwrite("frame.jpg", limiar_open)
	cv2.imwrite("frame2.jpg", limiar_close)
	cv2.imwrite("frame3.jpg", limiar)
	cv2.imwrite("frame32.jpg", or_img)

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
	print(objeto)
	cv2.imwrite("objeto.jpg", objeto)


	sift = cv2.xfeatures2d.SIFT_create()
	FLANN_INDEX_KDITREE = 0
	flannParam = dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
	flann = cv2.FlannBasedMatcher(flannParam,{})

	trainKP,trainDesc = sift.detectAndCompute(objeto,None)


	return trainKP, trainDesc 
