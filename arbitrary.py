#!/usr/bin/env python

import rospy, sys
from ras_lab1_msgs.msg import PWM


def arbitrary(left, right):

    # publisher
    pub = rospy.Publisher('/kobuki/pwm', PWM)

    # node
    rospy.init_node('arbitrary')

    # signal rate
    rate = rospy.Rate(10)

    # signal
    sig = PWM()

    # left and right wheel signals
    sig.PWM1, sig.PWM2 = int(left), int(right)

    while not rospy.is_shutdown():

        pub.publish(sig)

        rate.sleep()

if __name__ == "__main__":

    try:
        arbitrary(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass
