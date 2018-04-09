def segue():
	vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
	if len(media_objeto) != 0 and len(centro_objeto) != 0:
		dif_x = media_objeto[0]-centro_objeto[0]
		dif_y = media_objeto[1]-centro_objeto[1]
		if math.fabs(dif_x)<30: # Se a media_objeto estiver muito proxima do centro_objeto anda para frente
			vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
		else:
			if dif_x > 0: # Vira a direita
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
			else: # Vira a esquerda
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
		velocidade_saida.publish(vel)
		rospy.sleep(0	0