#!/usr/bin/env python3
#take initial controller funciton 

#make switch statements to change the callback to do what we want it to do
#distance from signs distance info, need to call from camera, will need to take inputs 
#will need to include all applicable callback functions 
import sys
import rospy, dlib, math #, cv2, argparse
from geometry_msgs.msg import Twist
from squaternion import Quaternion
from sensor_msgs.msg import Imu, Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from apriltag_ros.msg import AprilTagDetectionArray

# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180.0/math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class Controller:
    # computer vision constants
    FOCAL = 1961.5384
    STOP_WIDTH = 0.132 #actual width in centimeters

    # imu constants
    K_HDG = 0.1 # rotation controller constant
    HDG_TOL = 5 # heading tolerance +/- degrees
    MIN_ANG_Z = 0.0 # limit rad/s values sent to Turtlebot3
    MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3
    DISTANCE = 2.5 # distance from the wall to stop
    K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall
    MIN_LIN_X = 0.0 # limit m/s values sent to Turtlebot3
    MAX_LIN_X = 0.2 # limit m/s values sent to Turtlebot3
    LEFT_DIST=0  #DISTANCE FROM LEFT WALL
    RIGHT_DIST=0 #DISTANCE FROM RIGHT WALL

    # global variables, not updated often
    turn360 = False
    run_num_last = 0
    run_num = 0
    turn_count = 0
    tag_id = -1

    # class init
    def __init__(self):
        #class variables
        self.ctrl_c = False
        self.imu_msg = Twist()
        self.curr_yaw = 0
        self.goal_yaw = 0
        self.turning = False
        self.avg_dist_left = 0
        self.avg_dist_right = 0
        self.avg_dist_front = 0
        self.got_avg_left = False 
        self.got_avg_right = False 
        self.got_avg_front = False  
        self.stop_detected = False 
        self.tag_detected = False 
        self.stop_distance = -1
        self.stop_detected = False
        self.run_num_last = 0
        self.linear_vel = 0

        # publishers and subscribers
        self.imu_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.stop_sub = rospy.Subscriber('/stop_dist', Float32, self.stop_dist_controller)
        self.april_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apr_tag_camera_callback)
        self.imu_sub = rospy.Subscriber('imu', Imu, self.callback_imu)
        self.LIDAR_sub = rospy.Subscriber('scan', LaserScan, self.callback_lidar)

        rospy.Timer(rospy.Duration(.05),self.callback_controller)

        rospy.on_shutdown(self.shutdownhook)

    # funct to convert yaw to 360 values    
    def convert_yaw (self, yaw):
        return 360 + yaw if yaw < 0 else yaw	

    # receive imu data
    def callback_imu(self, imu):
        if not self.ctrl_c:
            # create a quaternion using the x, y, z, and w values
            # from the correct imu message
            q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)

            # convert the quaternion to euler in degrees
            e = q.to_euler(degrees=True)
            
            # get the yaw component of the euler
            self.curr_yaw = e[2]

            # convert yaw from -180 to 180 to 0 to 360
            self.curr_yaw = self.convert_yaw(self.curr_yaw)

    # controller to process all of the data received and tell the robot what to do
    def callback_controller(self, event):
        # local variables do not need the self
        yaw_err = 0
        ang_z = 0
        self.run_num += 1

        #print statements for debugging
        # print("stop dist      %.3f" % self.stop_distance)
        # print("april tag id:  %d"   % self.tag_id)
        # print("linear vel:    %.3f" % self.linear_vel)
        # print("curr yaw:      %.3f" % self.curr_yaw)
        # print("goal yaw:      %.3f" % self.goal_yaw)
        # print("turning:       %r"   % self.turning)
        # print("turn360:       %r"   % self.turn360)
        # print("turn_count360: %d"   % self.turn_count)
        # sys.stdout.flush()

        #stop detected state
        if self.stop_detected:
            #print statements for debugging
            # print("stop dist:     %.3f" % self.stop_distance)
            # print("run_num_last:    %d"   % self.run_num_last)
            # print("run_num:         %d"   % self.run_num)
            # sys.stdout.flush()
            if (self.run_num_last == (self.run_num - 175)): #time every .05sec, hold for 5sec and accounting for lag
                self.stop_detected=False
                linear_vel = 10
                ang_z = 0
            else:
                linear_vel = 0
                ang_z = 0

        # turn 360 state -- does not work because turn isnt here and we did not finish debugging
        # elif self.turn360:
        #     linear_vel = 0
        #     if self.turn_count == 0:
        #         self.turn_count += 1
        #         self.goal_yaw = self.curr_yaw + 120
        #     elif (abs(self.curr_yaw - self.goal_yaw) < 5):
        #         self.turn_count += 1
        #         self.goal_yaw = self.curr_yaw + 130
        #     if self.turn_count == 2:
        #         self.turn360 = False
        #         self.goal_yaw = self.curr_yaw
        #         self.turning = False
                # self.turn_count = 0  
        
        #tag detected state - we removed most of the distances because of the lag in the robot, it tended to meet specs anyway
        elif self.tag_detected:
            linear_vel = 0
            ang_z = 0
            # tag 3 state to initiate turn
            if self.tag_id==3:
                linear_vel=0
                ang_z = 0
                self.turn360 = True
                self.turn_count = 0
                self.goal_yaw= self.curr_yaw
                self.turning=True

            # turn 180 state
            if self.tag_id== 2:
                    linear_vel=0
                    self.goal_yaw = self.curr_yaw - 180 #left turn
                    self.turning = True

            # left turn state
            if self.tag_id== 1:
                linear_vel = 0
                self.goal_yaw = self.curr_yaw - 90 #left turn
                self.turning = True

            # right turn state
            if self.tag_id== 0:
                linear_vel = 0
                self.goal_yaw = self.curr_yaw + 90 #left turn
                self.turning = True

        # maintain speed if no stop or april tag
        else:
            linear_vel = 10

            # not turning -> wall following
            if not self.turning:
                    
                # Limit the linear speed of the robot to MIN_LIN_X and MAX_LIN_X.
                if linear_vel < self.MIN_LIN_X:
                    linear_vel = self.MIN_LIN_X
                elif linear_vel > self.MAX_LIN_X:
                    linear_vel = self.MAX_LIN_X

                # # check boundsturning
                if self.goal_yaw < 0:
                    self.goal_yaw += 360
                elif self.goal_yaw > 360:
                    self.goal_yaw -= 360

                # intersection state
                if not self.avg_dist_right and not self.avg_dist_left and self.avg_dist_front<=2:
                    ang_z = 0
                    linear_vel=0
                # door ignore
                elif (abs(self.avg_dist_left-self.avg_dist_right) > 1 or (self.got_avg_left and not self.got_avg_right) or (not self.got_avg_left and self.got_avg_right)):
                    ang_z = 0
                # wall following
                # robot is on right side of hallway
                elif (self.avg_dist_left-self.avg_dist_right > .05):
                    ang_z = -.5*(self.avg_dist_left-self.avg_dist_right)
                # robot is on left side of the hallway
                elif (self.avg_dist_right-self.avg_dist_left > .05):
                    ang_z = -.5*(self.avg_dist_left-self.avg_dist_right)
                # robot is centered
                else:
                    ang_z = 0
                    linear_vel = 10

            # turn until goal is reached
            elif self.turning:
                if self.goal_yaw < 0:
                    self.goal_yaw += 360
                elif self.goal_yaw > 360:
                    self.goal_yaw -= 360

                # proportional turn controller
                yaw_err = self.goal_yaw - self.curr_yaw
                ang_z = self.K_HDG * yaw_err
                
                # angular velocity cannot be greater or less than boundaries
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
                    linear_vel=0

        # set twist message and publish
        self.imu_msg.linear.x = linear_vel
        self.imu_msg.angular.z = ang_z
        self.imu_pub.publish(self.imu_msg)

    #will return the number of the apriltag that was detected (if detected)
    def apr_tag_camera_callback(self, data):
        self.tag_detected = False
        for tag in data.detections:
            self.tag_detected = True
            if tag.id[0]==3:
                self.tag_id=3
            if tag.id[0]== 2:
                self.tag_id=2
            if tag.id[0]== 1:
                self.tag_id=1
            if tag.id[0]== 0:
                self.tag_id=0 

    # receive lidar data
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
            
            #init local variables
            left_sum = 0
            right_sum = 0
            front_sum = 0
            count_left = 0
            count_right = 0
            count_front = 0

            # python way to iterate two lists at once!
            for deg, rng in zip(degrees, ranges):
                # sum and count the ranges 10 degrees off the nose of the robot
                # sum += rng -> if deg > 10: count += 1 
                if deg > 260 and deg < 280:
                    left_sum += rng
                    count_left += 1
                elif deg > 80 and deg < 100:
                    right_sum += rng
                    count_right += 1
                elif deg > 350 and deg < 10:
                    front_sum += rng
                    count_front += 1
                
            # ensure you don't divide by 0 and print average off the nose
            if count_left != 0:
                self.avg_dist_left = float(left_sum/count_left)
                self.got_avg_left = True
            else:
                self.got_avg_left = False

            if count_right != 0:
                self.avg_dist_right = float(right_sum/count_right)
                self.got_avg_right = True
            else:
                self.got_avg_right = False

            if count_front != 0:
                self.avg_dist_front = float(front_sum/count_front)
                self.got_avg_front = True
            else:
                self.got_avg_front = False

    # receive stop dist data from program on robot
    def stop_dist_controller(self, data):
        self.stop_distance = data.data
        if self.stop_distance > 0:
            linear_vel = 0
            self.stop_detected = True
            self.run_num_last = self.run_num

    # shutdown sequence
    def shutdownhook(self):
        self.ctrl_c = True
        # force the linear x and angular z commands to 0 before halting
        self.imu_msg.linear.x = 0
        self.imu_msg.angular.z = 0
        self.imu_pub.publish(self.imu_msg)        

# main!
if __name__ == '__main__':
	rospy.init_node('controller')
	c = Controller()
	rospy.spin()