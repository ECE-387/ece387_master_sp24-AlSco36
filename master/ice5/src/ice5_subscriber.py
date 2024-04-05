#!/usr/bin/env python3
import rospy
from ice5.msg import Person # import the message: from PKG.msg import MSG

class Listener:
    """Listener class that prints information about person"""
    def __init__(self):
        # TODO: create the subscriber that receives Person messages over the person topic
        # and calls the callback_person() function.
        self.sub= rospy.Subscriber('person', Person, self.callback_person)
        
        # nicely handle shutdown (Ctrl+c)
        rospy.on_shutdown(self.shutdownhook)
        
    def callback_person(self, person):
        # TODO: print the information about the person
        print("%s %s is %d years old!" % (person.firstname, person.lastname, person.age))
        
    def shutdownhook(self):
        print("Shutting down subscriber.")
        
if __name__ == '__main__':
    rospy.init_node('listener')
    # create an instance of the class
    Listener()
    # keeps python from exiting until this node is stopped
    rospy.spin()

