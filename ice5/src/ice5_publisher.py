#!/usr/bin/env python3
import rospy
from ice5.msg import Person # import the message: from PKG.msg import MSG

class Talker:
    """Class that publishes basic information about person"""
    def __init__(self, first = "Cadet", last = "Snuffy", age = 21):
        self.msg = Person()         # creates a Person message
        self.msg.firstname = first  # assign the firstname field
        self.msg.lastname = last    # assign the lastname field
        self.msg.age = age          # assign the age field
        
        # TODO: create the publisher that publishes Person messages over the person topic
        # Since we don't care about losing messages we can set the queue size to 1
        self.pub =rospy.Publisher('person', Person, queue_size=1)
        
        # TODO: create a timer that will call the callback_publish() function every .1 seconds (10 Hz)
        rospy.Timer(rospy.Duration(.1), self.callback_publish)
        
        # nicely handle shutdown (ctrl+c)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def callback_publish(self, event):
        if not self.ctrl_c:
            # TODO: publish the msg
            self.pub.publish(self.msg)
            
    def shutdownhook(self):
        print("Shutting down publisher.")
        self.ctrl_c = True
        
if __name__ == '__main__':
    rospy.init_node('talker')
    
    # create an instance of the Talker class changing the class variables
    Talker("Steven", "Beyer", 33)
    rospy.spin()	# run forever until ctrl+c    

