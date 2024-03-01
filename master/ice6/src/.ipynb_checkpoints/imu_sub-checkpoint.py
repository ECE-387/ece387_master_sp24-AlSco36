#!/usr/bin/env python3
import rospy
from squaternion import Quaternion
# TODO: import message type sent over imu topic
from sensor_msgs.msg import Imu

class IMU:
    """Class to read orientation data from Turtlebot3 IMU"""
    def __init__(self):        
        # TODO: subscribe to the imu topic that is published by the
        # Turtlebot3 and provides the robot orientation
        self.sub=rospy.Subscriber('imu', Imu, self.callback_imu)

        # nicely handle shutdown (Ctrl+c)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)                

    # The IMU provides yaw from -180 to 180. This function
    # converts the yaw (in degrees) to 0 to 360
    def convert_yaw (self, yaw):
        return 360 + yaw if yaw < 0 else yaw		

    # Print the current Yaw
    def callback_imu(self, imu):
        if not self.ctrl_c:
            # TODO: create a quaternion using the x, y, z, and w values
            # from the correct imu message
            q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)

            # TODO: convert the quaternion to euler in degrees
            e = q.to_euler(degrees=True)
            
            # TODO: get the yaw component of the euler
            yaw =e[2]

            # convert yaw from -180 to 180 to 0 to 360
            yaw = self.convert_yaw(yaw)
            print("Current heading is %f degrees." % (yaw))

    # clean shutdown
    def shutdownhook(self):
        print("Shutting down the IMU subscriber")
        self.ctrl_c = True

if __name__ == '__main__':
    rospy.init_node('imu_sub')
    IMU()
    rospy.spin()


