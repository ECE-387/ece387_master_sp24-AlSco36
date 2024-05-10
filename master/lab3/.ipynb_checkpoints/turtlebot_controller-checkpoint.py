#!/usr/bin/env python3

#TODO 1 Modify this header so that the correct information is displayed
#Name: Erin Clothier, Anna Scofield
#Name of code:turtlebot_controller.py
#For lab1, this will subscribe to mouse_client and publish to cmd_vel
#Will convert messages of type MouseController to Twist
#Deactivates when mouse wheel is scrolled up 
#last modified 11 Mar 2024


import rospy, math
from geometry_msgs.msg import Twist
from squaternion import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class Controller:
    """Class that controls subsystems on Turtlebot3"""
    K_HDG = 0.1 # rotation controller constant
    HDG_TOL = 10 # heading tolerance +/- degrees
    MIN_ANG_Z = 0.5 # limit rad/s values sent to Turtlebot3
    MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3
    DISTANCE = 0.4 # distance from the wall to stop
    K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall
    MIN_LIN_X = 0.05 # limit m/s values sent to Turtlebot3
    MAX_LIN_X = 0.2 # limit m/s values sent to Turtlebot3


    def __init__(self):
        #TODO 3 initialize the appropriate Controller class attributes
        #global activate
        self.msg = Twist()
        self.curr_yaw = 0
        self.goal_yaw = 0
        self.turning = False
        self.avg_dist = 0
        self.got_avg = False 

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.Timer(rospy.Duration(.2),self.callback_controller)
        self.imuSub = rospy.Subscriber('imu', Imu, self.callback_imu)
        self.LIDARSub = rospy.Subscriber('scan', LaserScan, self.callback_lidar)

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
        dist_error = 0
        linear_vel = 0
    
        # not turning, so get user input
        if not self.turning:

            # calculate the distance error and use that to drive your robot straight at a proportional rate 
            if self.got_avg == True:
                dist_error = self.avg_dist - self.DISTANCE
                linear_vel = self.K_POS * dist_error

                # Limit the linear speed of the robot to MIN_LIN_X and MAX_LIN_X.
                if linear_vel < self.MIN_LIN_X:
                    linear_vel = self.MIN_LIN_X
                elif linear_vel > self.MAX_LIN_X:
                    linear_vel = self.MAX_LIN_X

                #If within DISTANCE of a wall, then stop and start turning (left or right, you decide).
                if dist_error < 0:
                    linear_vel = 0
                    self.goal_yaw = self.curr_yaw + 90 #left turn
                    self.turning = True

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
        self.msg.linear.x = linear_vel
        self.msg.angular.z = ang_z
        self.pub.publish(self.msg)

    def callback_lidar(self, scan):
        if not self.ctrl_c:
            degrees = []
            ranges = []
            
            # determine how many scans were taken during rotation
            count = len(scan.ranges)
            
            for i in range(count):
                # using min angle and incr data determine curr angle, 
                # convert to degrees, convert to 360 scale
                degrees.append(int(DEG_CONV(RAD2DEG(scan.angle_min + scan.angle_increment*i))))
                rng = scan.ranges[i]
                
                # ensure range values are valid; set to 0 if not
                if rng < scan.range_min or rng > scan.range_max:
                    ranges.append(0.0)
                else:
                    ranges.append(rng)
            
            #we added... not sure... 
            sum = 0
            count_off30 = 0

            # python way to iterate two lists at once!
            for deg, rng in zip(degrees, ranges):
                # TODO: sum and count the ranges 30 degrees off the nose of the robot - HERE!!!!!!!
                # sum += rng
                # if deg > 30: count_off30 += 1
                if deg > 345 or deg < 15:
                    sum += rng
                    count_off30 += 1
                
            # TODO: ensure you don't divide by 0 and print average off the nose
            if count_off30 != 0:
                #print("average: %f" % float(sum/count_off30))
                self.avg_dist = float(sum/count_off30)
                self.got_avg = True
            else:
                self.got_avg = False

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