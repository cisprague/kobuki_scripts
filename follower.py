#!/usr/bin/env python

import rospy, sys
import numpy as np
from ras_lab1_msgs.msg import PWM
from ras_lab1_msgs.msg import Encoders
from ras_lab1_msgs.msg import ADConverter
from geometry_msgs.msg import Twist

class follower(object):

    def __init__(self, d, v, kp, ki):

        # parametres
        self.tpr = 360
        self.b   = 0.115
        self.r   = 0.0352
        self.l   = 0.2

        # desired distance and velocity
        self.d, self.v = float(d), float(v)

        # controller gains
        self.kp, self.ki = float(kp), float(ki)

        # integral errors
        self.int1, self.int2 = 0, 0

        # initialise node
        rospy.init_node('follower')

        # publisher
        self.pub = rospy.Publisher('kobuki/pwm', PWM)

        # subscriber
        rospy.Subscriber('/kobuki/adc', ADConverter, self.callback1)


        rospy.spin()

    def callback1(self, data):

        # signals
        s1, s2 = float(data.ch1), float(data.ch2)

        # distances
        d1, d2 = 1.114*np.exp(-0.004*s1), 1.114*np.exp(-0.004*s2)

        # orientation
        theta = np.arctan((d1 - d2)/np.sqrt((d1 - d2)**2 + self.l**2))

        # distance
        d = (d1 + d2)/2

        # errors
        et = theta
        ed = self.d - d
        ev = self.v - 

        print(theta, d)


if __name__ == '__main__':

    follower(0.1, 0.5, 10, 10)
