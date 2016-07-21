#!/usr/bin/env python
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import threading


class ColorTracker:
    def __init__(self, debugging):
        self.node_name = "ColorTracker"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)

        self.pub_detection = rospy.Publisher("/detection", Int32MultiArray, queue_size=10)

        self.debugging = debugging

        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def detection(self, img):
	bottom_dim = img.shape[0]
	img = img[bottom_dim-75:, :, :]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([25, 60, 90])
        upper = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        mask = cv2.erode(mask, (3,3), iterations=1)

	#return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	print(len(contours))

        if self.debugging:
            cv2.drawContours(img, contours, -1, (0, 0, 255), 2)
        
        for i in range(0, len(contours)):
            c = contours[i]
            area = cv2.contourArea(c)
            if area < 600: # minimum area threshold
                continue

            perim = cv2.arcLength(c, True) # perimeter
            approx = cv2.approxPolyDP(c, 0.05 * perim, True)

            if len(approx) != 4:
                continue


            if self.debugging:
                cv2.drawContours(img, [c], -1, (255, 0, 0), 3) 
                cv2.drawContours(img, [approx], -1, (0, 255, 0), 5) 

                coord = (approx[0][0][0], approx[0][0][1])
                cv2.putText(img, "GREEN", coord, cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255),  2)

            M = cv2.moments(approx)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            if self.debugging:
                cv2.circle(img, (cx, cy), 10, (255, 255, 255), -1)

            approx_area = cv2.contourArea(approx)

            self.publish_detected(cx, cy, approx_area)


        return img


    def publish_detected(self, cx, cy, approx_area):
        msg = Int32MultiArray()
        msg.data = [approx_area, cx]
        self.pub_detection.publish(msg)

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        output = self.detection(image_cv)
        
        if self.debugging:
            try:
                self.pub_image.publish(\
                        self.bridge.cv2_to_imgmsg(output, "bgr8"))
            except CvBridgeError as e:
                print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('ColorTracker')
    e = ColorTracker(True)
    rospy.spin()

