#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading


class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def detection(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([20, 215, 170])
        upper = np.array([29, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        #return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        mask = cv2.erode(mask, (3,3), iterations=1)
        # mask = cv2.GaussianBlur(mask, (11, 11), 0)

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for i in range(0, len(contours)):
            c = contours[i]
            area = cv2.contourArea(c)
            if area < 600: # minimum area threshold
                continue

            perim = cv2.arcLength(c, True) # perimeter
            approx = cv2.approxPolyDP(c, 0.05 * perim, True)

            if len(approx) != 4:
                continue


            print("Approximation length: ", len(approx))

            cv2.drawContours(img, [c], -1, (255, 0, 0), 3) 
            cv2.drawContours(img, [approx], -1, (0, 255, 0), 5) 

            coord = (approx[0][0][0], approx[0][0][1])
            cv2.putText(img, "YELLOW", coord, cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255),  2)


        return img


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        output = self.detection(image_cv)
        
        try:
            self.pub_image.publish(\
                    self.bridge.cv2_to_imgmsg(output, "bgr8"))
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

