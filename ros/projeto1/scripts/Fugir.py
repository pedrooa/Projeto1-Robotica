def foge():
	vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

	if len(media_madfox) != 0 and len(centro_madfox) != 0 and achou_madfox == 1:
		dif_x = media_madfox[0]-centro_madfox[0]
		dif_y = media_madfox[1]-centro_madfox[1]
	if dif_x < 0: # Vira a esquerda
		vel = Twist(Vector3(0,0,0), Vector3(0,0,-5))
		time.sleep(0.7)
		print('esquerda')
	if dif_x > 0: # Viraa direita
			vel = Twist(Vector3(0,0,0), Vector3(0,0,5))
			time.sleep(0.7)
	else: 
		print('nao achou_madfox')
		vel = Twist(Vector3(0.01,0,0), Vector3(0,0,0))
		velocidade_saida.publish(vel)
		rospy.sleep(0.01)