#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32MultiArray


class Control:

    def __init__(self):
        self.sub = rospy.Subscriber("/detection", Int32MultiArray, self.scan_recieved)
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

        #where we want the centroid to be in relation to the screen
        self.x_des = 640
        #desired area of object on screen
        self.area_des = 
        #how accurate the centroid is from our current position
        self.centroid_threshhold = 
        #how accurate the area is from the current area of the object
        self.area_threshhold = 
        #initial steering angle
        self.steering_angle = 0
        #initial speed
        self.speed = 0
        #p constant for pid controll
        self.K_p = 

    def angle_control(self, centroid_cooor):
        centroid_errorx = centroid_coor - self.x_des
        if abs(centroid_errorx) <= self.centroid_threshhold:
            steering_angle = 0
        else:
            p = self.K_p * centroid_errorx
            steering_angle = p
        return steering_angle

    def speed_control(self, area)
        area_error = area - self.area_des
        if abs(area_error) <= self.area_threshhold:
            speed = 0
        elif area_error < 0:
            speed = 1.0
        else:
            speed = -1.0
        return speed

    def scan_recieved(self, msg):
        self.drive(
            self.speed_control(msg[0]),
            self.angle_control(msg[1])
        )

    def drive(self, speed, steering_angle):
        out = AckermannDriveStamped()
        out.drive.speed = speed
        out.drive.steering_angle = steering_angle

        self.pub.Publish(out)

if __name__=="__main__":
    rospy.init_node("Control")
    node = Control()
    rospy.spin()
