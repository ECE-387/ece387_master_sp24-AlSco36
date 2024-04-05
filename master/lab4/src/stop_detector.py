#!/usr/bin/env python3
import rospy, cv2, dlib
from cv_bridge import CvBridge, CvBridgeError

# TODO: import usb_cam message type
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class StopDetector(object):

    FOCAL = 1961.5384
    STOP_WIDTH = 0.132 #actual width in centimeters

    def __init__(self, detectorLoc):
        self.ctrl_c = False

        #TODO: create subscriber to usb_cam image topic
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)

        #distance publisher
        self.msg = Float32()
        self.pub = rospy.Publisher('stop_dist', Float32, queue_size = 1)
        rospy.Timer(rospy.Duration(.1),self.publish_distance)

        self.bridge_object = CvBridge()
        self.detector = dlib.simple_object_detector(detectorLoc)

        self.width = int()

        rospy.on_shutdown(self.shutdownhook)

    def camera_callback(self, data):
        if not self.ctrl_c:
            #TODO:
            #write code to get ROS image
            img = data
            #convert to OpenCV image
            try:
                self.cv_image = self.bridge_object.imgmsg_to_cv2(img, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
            #apply detector
            self.boxes = self.detector(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB))
            # if(self.boxes):
            #     print("Stop Detected!")
            #add boxes to image
            for b in self.boxes:
               (x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())
               cv2.rectangle(self.cv_image, (x, y), (w, h), (0, 255, 0), 2)
               self.width = w
            #    print(x, y, w, h)
            #    distance = self.STOP_WIDTH*self.FOCAL/w
            #    print(distance)
            # display image
            cv2.imshow('image', self.cv_image)
            cv2.waitKey(1)

    def publish_distance(self, event):
            if(self.width):
                distance = self.STOP_WIDTH*self.FOCAL/self.width
                # print(distance)
                self.msg = distance
                self.pub.publish(self.msg)

    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('stop_detector')
    detector = rospy.get_param("/stop_detector/detector")
    stop_detector = StopDetector(detector)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

