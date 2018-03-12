#!/usr/bin/env python

import rospy, sys
from ras_lab1_msgs.msg import PWM
from geometry_msgs.msg import Twist

def callback(data):
    print(data)

def cartesian(v, w):

    # publisher
    pub = rospy.Publisher('/kobuki/pwm', PWM)

    # subscriber
    rospy.Subscriber('/mobile_base/commands/velocity', Twist, callback)

    # node
    rospy.init_node('cartesian')

    rospy.spin()

if __name__ == '__main__':
    cartesian(5, 5)
