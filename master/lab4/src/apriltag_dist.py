#!/usr/bin/env python3
import rospy, cv2, dlib
from cv_bridge import CvBridge, CvBridgeError

# TODO: Import the appropriate AprilTag message
from apriltag_ros.msg import AprilTagDetectionArray

class TagDetector():

    FOCAL = 1131
    TAG_WIDTH = 13 #actual width in centimeters

    def __init__(self):
        self.ctrl_c = False
        #Subscribe to the tag_detections topic
        self.sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.camera_callback)

        rospy.on_shutdown(self.shutdownhook)

    def camera_callback(self, data):
        for tag in data.detections:
            print("Tag detected!")
            print("Tag ID: %.0f" % tag.id)
            print("distance: %.3fm\n" % tag.pose.pose.pose.position.z )

    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('apriltag_dist')
    tag_detector = TagDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass