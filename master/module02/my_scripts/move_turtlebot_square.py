#!/usr/bin/env python3
import rospy, time, math
from geometry_msgs.msg import Twist

class MoveTurtleBot():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)    # 10 Hz
        
    def publish_cmd_vel_once(self):
        """
        In case publishing fails on first attempt
        """
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
                
    def shutdownhook(self):
        rospy.loginfo("Shutting down. Stopping TurtleBot!")
        self.stop_turtlebot()
        self.ctrl_c = True
        
    def stop_turtlebot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_cmd_vel_once()
        
    def move_time(self, moving_time = 10.0, lin_spd = 0.2, ang_spd = 0.2):
        self.cmd.linear.x = lin_spd
        self.cmd.angular.z = ang_spd
        
        self.publish_cmd_vel_once()
        time.sleep(moving_time)
        
    def move_square(self):
        i = 0
        
        while not self.ctrl_c:
            # Move Forward
            self.move_time(moving_time = 2.0, lin_spd = 0.2, ang_spd = 0)
            # Turn
            ang_spd = 0.5    # rad/sec
            moving_time = math.radians(90)/ang_spd
            self.move_time(moving_time = moving_time, lin_spd = 0.0, ang_spd = ang_spd)
            
        
if __name__ == '__main__':
    rospy.init_node('move_turtlebot')
    move_object = MoveTurtleBot()
    try:
        move_object.move_square()
    except rospy.ROSInterruptException:
        pass

