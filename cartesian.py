#!/usr/bin/env python

import rospy, sys
import numpy as np
from ras_lab1_msgs.msg import PWM
from ras_lab1_msgs.msg import Encoders
from geometry_msgs.msg import Twist

class cartesian(object):

    def __init__(self, v, w):

        # desired linear and angular velocity
        self.v, self.w = v, w

        # publisher
        pub = rospy.Publisher('kobuki/pwm', PWM)

        # subscriber
        rospy.Subscriber('/kobuki/encoders', Encoders, self.callback)

        # initialise node
        rospy.init_node('cartesian')

        rospy.spin()


    def callback(self, data):

        # left and right wheel signals
        de1, de2 = data.delta_encoder1, data.delta_encoder2

        # error
        ew1, ew2 = de1*2*np.pi*10/360, de2*2*np.pi*10/360

        print(ew1)


if __name__ == '__main__':
    cartesian(5, 5)
