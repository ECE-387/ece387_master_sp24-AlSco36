#!/usr/bin/env python3

#TODO 1 Modify this header so that the correct information is displayed
#Name: Erin Clothier, Anna Scofield
#Name of code:turtlebot_controller.py
#For lab1, this will subscribe to mouse_client and publish to cmd_vel
#Will convert messages of type MouseController to Twist
#Deactivates when mouse wheel is scrolled up 
#last modified 29 Feb 2024


import rospy
from geometry_msgs.msg import Twist
from squaternion import Quaternion
from sensor_msgs.msg import Imu

class Controller:
    """Class that controls subsystems on Turtlebot3"""
    K_HDG = 0.1 # rotation controller constant
    HDG_TOL = 10 # heading tolerance +/- degrees
    MIN_ANG_Z = 0.5 # limit rad/s values sent to Turtlebot3
    MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3

    def __init__(self):
        #TODO 3 initialize the appropriate Controller class attributes
        #global activate
        self.msg = Twist()
        self.curr_yaw = 0
        self.goal_yaw = 0
        self.turning=False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        rospy.Timer(rospy.Duration(.015),self.callback_controller)
        self.sub= rospy.Subscriber('imu', Imu, self.callback_imu)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def convert_yaw (self, yaw):
        return 360 + yaw if yaw < 0 else yaw	

    def callback_imu(self, imu):
        if not self.ctrl_c:
            # TODO: create a quaternion using the x, y, z, and w values
            # from the correct imu message
            q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)

            # TODO: convert the quaternion to euler in degrees
            e = q.to_euler(degrees=True)
            
            # TODO: get the yaw component of the euler
            self.curr_yaw = e[2]

            # convert yaw from -180 to 180 to 0 to 360
            self.curr_yaw = self.convert_yaw(self.curr_yaw)
            # print("Current heading is %f degrees." % (self.curr_yaw))


    def callback_controller(self, event):
        # local variables do not need the self
        yaw_err = 0
        ang_z = 0

        # not turning, so get user input
        if not self.turning:
            #read from user and set value to instance variable, self.goal_yaw
            user_input = input("Input l or r to turn 90 deg\n")
                
            # check input and determine goal yaw
            if user_input == "l":
                self.goal_yaw = self.curr_yaw + 90
                self.turning = True
            elif user_input == "r":
                self.goal_yaw = self.curr_yaw - 90
                self.turning = True
            else:
                print("No be better! Valid inputs are 'l' or 'r'")
                
            # check bounds
            if self.goal_yaw < 0:
                self.goal_yaw += 360
            
            elif self.goal_yaw > 360:
                self.goal_yaw -= 360
            
        # turn until goal is reached
        elif self.turning:
            yaw_err = self.goal_yaw - self.curr_yaw
            
            # determine if robot should turn clockwise or counterclockwise
            if yaw_err > 180:
                yaw_err = yaw_err - 360
            elif yaw_err < -180:
                yaw_err = yaw_err + 360
                
            # proportional controller that turns the robot until goal 
            # yaw is reached
            ang_z = self.K_HDG * yaw_err
            
            if ang_z > 0:
                if ang_z < self.MIN_ANG_Z:
                    ang_z = self.MIN_ANG_Z		
                elif ang_z > self.MAX_ANG_Z:
                    ang_z = self.MAX_ANG_Z	
                    
            # need to add negative test as well!
            if ang_z < 0:
                if abs(ang_z) < self.MIN_ANG_Z:
                    ang_z = -1 * self.MIN_ANG_Z		
                elif abs(ang_z) > self.MAX_ANG_Z:
                    ang_z = -1 * self.MAX_ANG_Z	

            # check goal orientation
            if abs(yaw_err) < self.HDG_TOL:
                self.turning = False
                ang_z = 0
                
        # set twist message and publish
        self.msg.linear.x = 0
        self.msg.angular.z = ang_z
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