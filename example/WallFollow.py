#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np


class WallFollow:

    def __init__(self):

        # State
        self.pid = {'p': 0.5, 'd': 1, 'i': 0}
        self.distance = {'desired': 1, 'last': 1, 'direction': 'left'}

        # Constants
        self.speed = 2
        self.collisionDistance = 1.5
        self.angles = {'left': 780, 'right': 300, 'sideRange': 10,
                       'frontRange': 15}

        # Publishers
        self.drivePub = rospy.Publisher(
            "vesc/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped, queue_size=5)

        # Listeners
        rospy.Subscriber("scan", LaserScan, self.scanReceived)
        rospy.Subscriber("joy", Joy, self.joyReceived)

    # Calculate Steering Angle
    def calculateAngle(self, scans):
        distance = self.onePointDistance(scans)
        print distance
        error = self.distance['desired'] - distance
        pee = -self.pid['p'] if self.distance['direction'] == 'left' else self.pid['p']
        p = pee * error
        d = self.pid['d'] * (distance - self.distance['last'])
        print "p:", p, "d:", d, "sum", p + d
        angle = p + d

        self.distance['last'] = distance
        return angle

    def onePointDistance(self, msg):
        scans = msg.ranges
        # Minimum lidar distance measure in range of 20
        minimum = min(scans[(self.angles[self.distance['direction']] -
                             self.angles['sideRange']):
                            (self.angles[self.distance['direction']] +
                             self.angles['sideRange'])])
        return minimum

    # The angle from beginning of right side.
    def get_angle(self, scans, array_pos):
        angle = array_pos*270/1080
        return angle

    def regressionDistance(self, msg):
        scans = msg.ranges
        x_coordinate_array = []
        y_coordinate_array = []
        # If the wall is on the right side
        if self.distance['direction'] == 'right':
            my_range = [180, 301]
        else:
            my_range = [900, 780]

        # Uses points between R60 and R90
        for i in range(my_range[0], my_range[1]):
            angle = self.get_angle(scans, i)
            z = scans[i]  # The hypotenous

            # Convert polar coordinates to cartesian coordinates
            x_coordinate_array.append(math.cos(angle)*z)
            y_coordinate_array.append(math.sin(angle)*z)

        line = np.polyfit(x_coordinate_array, y_coordinate_array, 1)
        reg_line_slope = line[0]  # Regression line slope
        reg_line_y_int = line[1]  # Regression line y-intercept
        perp_line_slope = -1/reg_line_slope	 # perpindicular line slope

        intersection_x = -(reg_line_y_int)/(1/reg_line_slope+reg_line_slope)
        intersection_y = perp_line_slope*intersection_x

        dist = (intersection_x**2 + intersection_y**2)**(1/2)

        reg_lin_x_int = -reg_line_y_int/reg_line_slope
        # Angle relative to the line perpendicular to the wall.
        # The angle will be negative if it is on the backside of the robot.
        angle = np.arctan(((reg_lin_x_int-intersection_x)**2 +
                           ((intersection_y)**2)**(1/2))/dist)

        return dist

    def trigDistance(self, msg):
        scans = msg.ranges
        angle1i = 60
        angle2i = 300  # +-30 degrees from being perpendicular to wall
        desiredrange = 1  # replace with distance['desired']

        driveAngle = 0

        r1 = scans[angle1i]
        r2 = scans[angle2i]

        # difference in angles
        da = math.radians(270.0) * ((angle2i-angle1i) / 1081.0)
        print da
        d = (r1*r2*math.sin(da))/((r1**2 + r2**2 - 2*r1*r2*math.cos(da))**.5)
        return d

    # Calculate Driving Speed
    def calculateSpeed(self, scans):
        return (self.speed if (min(scans[540 - self.angles['frontRange']:
                                         540 + self.angles['frontRange']]) <
                               self.collisionDistance)
                else self.speed)

    # Scan Callback
    def scanReceived(self, msg):
        self.drive(
            self.calculateAngle(msg),
            self.calculateSpeed(msg.ranges)
        )

    def joyReceived(self, msg):
        print msg

    # Construct & Send Ackermann Packet
    def drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle

        self.drivePub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("WallFollow")

    node = WallFollow()

    rospy.spin()
