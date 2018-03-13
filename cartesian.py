#!/usr/bin/env python

import rospy, sys
import numpy as np
from ras_lab1_msgs.msg import PWM
from ras_lab1_msgs.msg import Encoders
from geometry_msgs.msg import Twist

class cartesian(object):

    def __init__(self, v, w, kp, ki):

        # initialise node
        rospy.init_node('cartesian')

        # desired wheel rates
        self.vw1, self.vw2 = float(v) - 0.115*float(w), float(v) + 0.115*float(w)

        # integral errors
        self.int1, self.int2 = 0, 0

        # controller gains
        self.kp, self.ki = float(kp), float(ki)

        # publisher
        self.pub = rospy.Publisher('kobuki/pwm', PWM)

        # subscriber
        rospy.Subscriber('/kobuki/encoders', Encoders, self.callback)

        rospy.spin()


    def callback(self, data):

        # signals
        s1, s2 = data.delta_encoder1, data.delta_encoder2

        # estimated velocities
        r = 0.0352
        b = 0.115
        f = 10
        tpr = 360
        vw1, vw2 = 2*np.pi*r*f*s1/tpr, 2*np.pi*r*f*s2/tpr

        # errors
        e1, e2 = self.vw1 - vw1, self.vw2 - vw2

        # integration error
        self.int1 += e1*0.1
        self.int2 += e2*0.1

        # control signals
        sig = PWM()
        sig.PWM1 = self.kp*e1 + self.ki*self.int1
        sig.PWM2 = self.kp*e2 + self.ki*self.int2

        #print(e1, e2)

        self.pub.publish(sig)



if __name__ == '__main__':

    try:
        cartesian(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    except rospy.ROSInterruptException:
        pass
