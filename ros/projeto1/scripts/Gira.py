def gira():
	ang_speed = 0.4
	vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
	velocidade_saida.publish(vel)