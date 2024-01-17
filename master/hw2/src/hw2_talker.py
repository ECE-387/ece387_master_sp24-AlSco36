#!/usr/bin/env python 
import rospy 
from std_msgs.msg import String 

def talker(): 
	pub= rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker',anonymous =False)
	rate= rospy.Rate(10) #10 Hz
	cnt=0
	while not rospy.is_shutdown():
		#create variable to store basic string publish 
		chat_str = "Hello World, this is my first ROS node."
		#concat string to message to show how # times published 
		chat_str = chat_str+ " This message published %s times " % cnt
		cnt= cnt+1 
		#publish string to chatter topic 
		pub.publish(chat_str)
		#pause for.1 sec based on rate set above 
		rate.sleep()

if __name__ == '__main__' :
	try: 
		talker() 
	except rospy.ROSInterruptException: 
		pass