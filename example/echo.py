#!/usr/bin/env python
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading 

#start a class called Echo
class Echo:
    def __init__(self):
        self.node_name = "Echo"
        #forces seperate process in computer to open so processes can run in parallel, doesnt have to wait for next message
        self.thread_lock = threading.Lock()
        #subscribes to camera with message type Image
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        #publishes to topic echo_image
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    #callback function
    def cbImage(self,image_msg):
        #makes thread for every image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        #if the parent dies image process doesnt die
        thread.setDaemon(True)
        thread.start()

    #callback inside of thread
    def processImage(self, image_msg):
        #if the thread is not ready, exit function
        if not self.thread_lock.acquire(False):
            return
        #makes an open cv representation of the image received
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        
        #if the thread is ready
        try:
            #publish image thats converted from cv2 to msg
            self.pub_image.publish(\
                    self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        #returns error
        except CvBridgeError as e:
            print(e)
        #stops thread once everythings done
        self.thread_lock.release()

#calling the main function
if __name__=="__main__":
    #starts node named echo thats of echo class
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

