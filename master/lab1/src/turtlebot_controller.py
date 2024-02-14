#!/usr/bin/env python3

#TODO 1 Modify this header so that the correct information is displayed
#Name: Erin Clothier, Anna Scofield
#Name of code:turtlebot_controller.py
#For lab1, this will subscribe to mouse_client and publish to cmd_vel
#Will convert messages of type MouseController to Twist
#Deactivates when mouse wheel is scrolled up 
#last modified 9 Feb 2024


import rospy
#TODO 2 Import the appropriate message types that we will need
from geometry_msgs.msg import Twist
from lab1.msg import MouseController

class Controller:
	"""Class that controls subsystems on Turtlebot3"""
	def __init__(self):
		#TODO 3 initialize the appropriate Controller class attributes
		#global activate
		self.msg = Twist()

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

		rospy.Subscriber('mouse_info', MouseController, self.callback_mouseControl)

		self.ctrl_c = False
		rospy.on_shutdown(self.shutdownhook)

	def callback_mouseControl(self, mouseInfo):
		print("reaches callback_mouseControl")
		#TODO 4 Scale xPos from -1 to 1 to -.5 to .5
		mouseInfo.xPos = mouseInfo.xPos/2.0
		#TODO 5 set angular z in Twist message to the scaled value in the appropriate direction
		self.msg.angular.z = -1*mouseInfo.xPos
		#TODO 6 Scale yPos from -1 to 1 to -.5 to .5
		mouseInfo.yPos = mouseInfo.yPos/2.0
		#TODO 7 set linear x in Twist message to the scaled value in the appropriate direction
		self.msg.linear.x = mouseInfo.yPos
		#TODO 8 publish the Twist message
		self.pub.publish(self.msg)


	def shutdownhook(self):
		print("Controller exiting. Halting robot.")
		self.ctrl_c = True
		#TODO 9 force the linear x and angular z commands to 0 before halting
		self.msg.linear.x = 0
		self.msg.angular.z = 0
		self.pub.publish(self.msg)


if __name__ == '__main__':
	rospy.init_node('controller')
	c = Controller()
	rospy.spin()