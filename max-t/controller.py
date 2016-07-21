#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan


class Control:
    def __init__(self):
        rospy.Subscriber("/detection", Int32MultiArray, self.detect_recieved)
        rospy.Subscriber("scan", LaserScan, self.scan_received)
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation",
                                   AckermannDriveStamped, queue_size=1)

        # where we want the centroid to be in relation to the screen
        self.x_des = 640
        # desired area of object on screen
        self.area_des = 120000
        # how accurate the centroid is from our current position
        self.centroid_threshhold = 20
        # how accurate the area is from the current area of the object
        self.area_threshhold = 80
        # initial steering angle
        self.steering_angle = 0.0
        # initial speed
        self.speed = 0.5
        # p constant for pid controll
        self.K_p = -0.0005

        self.state = "findWall"

    # calculate what the steering angle should be based on how far the centroid of the object is from the middle of the screen
    def angle_control(self, centroid_coor):
        centroid_errorx = centroid_coor - self.x_des
        # if the difference in centroid is small enough, don't have to change steering angle
        if abs(centroid_errorx) <= self.centroid_threshhold:
            steering_angle = 0
        # if not, use proportional control
        else:
            p = self.K_p * centroid_errorx
            steering_angle = p
        return steering_angle

    # calculate what the speed should be based on how big the object is in
    # relation to the screen
    def speed_control(self, area):
        area_error = area - self.area_des
        # if the error is less then thershhold then neglect
        if abs(area_error) <= self.area_threshhold:
            speed = 0
        # if the desired area is bigger than actual, have positive speed
        # and more towards the object
        elif area_error < 0:
            speed = self.speed
        else:
            speed = 0
        # otherwise move backwards
        return speed

    # callback function for recieving msgs from detection
    def detect_recieved(self, msg):
        self.drive(
            # pass in actual area, which is the first arg of the message, to change the speed
            self.speed_control(msg.data[0]),
            # pass in the x of the centroid of the obj to change the steering angle
            self.angle_control(msg.data[1])
        )

    def scan_received

    # callback function for driving
    def drive(self, speed, steering_angle):
        out = AckermannDriveStamped()
        out.drive.speed = speed
        out.drive.steering_angle = steering_angle

        self.pub.publish(out)

if __name__=="__main__":
    rospy.init_node("Control")
    node = Control()
    rospy.spin()
