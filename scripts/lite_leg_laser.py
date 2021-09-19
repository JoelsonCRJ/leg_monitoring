#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np
from sklearn.cluster import DBSCAN
import math
import time

class LegL():
	def __init__(self):
		self.rospy = rospy
		self.rospy.init_node('LegMonitoring', anonymous = True)
		self.rospy.loginfo("Starting Lower Limbs Monitoring")
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initVariables()
		self.mainControl()

	def initParameters(self):
		self.LegMonitoringTopic = self.rospy.get_param("~LegMonitoring_topic","/rp_scan")
		self.controlRate = self.rospy.get_param("~control_rate", 20)
		self.angle_min_new = self.rospy.get_param("~angle_min_new",155)
		self.angle_max_new = self.rospy.get_param("~angle_min_new",215)
		return

	def initPublishers(self):
		self.pubDist = self.rospy.Publisher("/distance", Float32, queue_size=10)
		self.pubDistF = self.rospy.Publisher("/distanceF", Float32, queue_size=10)
		return

	def initSubscribers(self):
		self.subPose = self.rospy.Subscriber(self.LegMonitoringTopic, LaserScan, self.callbackLaser)
		return

	def initVariables(self):
		self.rate = self.rospy.Rate(self.controlRate)
		self.angle_min = 0
		self.angle_max = 0
		self.scan_time = 0
		self.ranges = 0
		self.angle_increment = 0
		self.time_increment = 0
		self.range_min = 0
		self.range_max = 0
		self.change = False

		self.amostras = [[0,0],[0,0]]
		self.distAnt = 0
		self.distAtual = 0
		self.veloc = [0]
		self.mediaX = 0
		self.mediaY = 0
		self.k = 0
		"""----------------------------------  """

		#vetor com os parametros da formacao.
		#posicao 1 - a distancia entre os robos
		self.qDes = np.array([0.7, 0, 0, 0])

		return

	""" ---------------------------------- """
	def callbackLaser(self, msg):

		self.angle_min = msg.angle_min
		self.angle_max = msg.angle_max
		self.angle_increment = msg.angle_increment
		self.time_increment = msg.time_increment
		self.scan_time = msg.scan_time
		self.range_min = msg.range_min
		self.range_max = msg.range_max
		self.ranges = msg.ranges
		self.change = True

	""" ---------------------------------- """

	def legCluster(self):
		clustering = DBSCAN(eps=0.05, min_samples=4).fit(self.cart)
		IDX = clustering.labels_
		k = np.max(IDX)


		#gera a variavel das medias
		medias = []
		for j in range(k+1):
			Xj = self.cart[IDX==j]
			Xj[:,0]=Xj[:,0] 
			mediaX = np.mean(Xj[:,0])
			mediaY = np.mean(Xj[:,1])
			medias.append([mediaX,mediaY])


		if medias != []:
			mediasArray = np.array(medias)
			self.mediaX = np.mean(mediasArray[:,0])
			self.mediaY = np.mean(mediasArray[:,1])

			self.amostras[1]=[self.mediaX, self.mediaY]
			self.distAnt = math.sqrt(math.pow(self.amostras[0][0],2)+math.pow(self.amostras[0][1],2))
			self.distAtual = math.sqrt(math.pow(self.amostras[1][0],2)+math.pow(self.amostras[1][1],2))
			self.end = time.time()
			tempo = (self.end-self.start)
			self.veloc.append((self.distAtual-self.distAnt)/(tempo*10))

			self.amostras[0] = self.amostras[1]

			msg1 = Float32()
			msg1.data = self.mediaX
			self.pubDist.publish(msg1)
			msg2 = Float32()
			msg2.data = self.qDes[0]
			self.pubDistF.publish(msg2)
		

		return

	def mainControl(self):
		while not self.rospy.is_shutdown():
			if self.change:
				self.start = time.time()
				self.cartesiano = []
				angulo = self.angle_min
				anguloMin = self.angle_min_new
				anguloMax = self.angle_max_new
				vetorRangeCropped = [anguloMin*4, anguloMax*4]

				
				incr = self.angle_increment
				for i in range(len(self.ranges)):
					if not math.isnan(self.ranges[i]) and not math.isinf(self.ranges[i]) and ((i> vetorRangeCropped[0] and i< vetorRangeCropped[1])) and self.ranges[i]<0.7:
						self.cartesiano.append([self.ranges[i] * math.cos(angulo), self.ranges[i] * math.sin(angulo)])
					angulo = angulo + incr
					

				self.cart = np.array(self.cartesiano)

				if self.cartesiano != []:
					self.legCluster()

				else:
					msg1 = Float32()
					msg1.data = 0.0
					self.pubDist.publish(msg1)

				self.change = False

			self.rate.sleep()

if __name__ == '__main__':
	try:
		legL = LegL()
	except rospy.ROSInterruptException:
		pass
