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
        self.f   = 10

        # desired distance and velocity
        self.d, self.v = float(d), float(v)

        # controller gains
        self.kp, self.ki = float(kp), float(ki)

        # integral errors
        self.int1, self.int2 = 0, 0

        # initialise node
        rospy.init_node('follower')

        # publisher
        self.pub_pwm = rospy.Publisher('kobuki/pwm', PWM)

        # subscribers
        rospy.Subscriber('/kobuki/adc', ADConverter, self.cb_adc)
        rospy.Subscriber('/kobuki/encoders', Encoders, self.cb_enc)

        rospy.spin()

    def cb_adc(self, data):

        # signals
        s1, s2 = float(data.ch1), float(data.ch2)

        # distances
        self.d1, self.d2 = 1.114*np.exp(-0.004*s1), 1.114*np.exp(-0.004*s2)

        # orientation
        #self.mtheta = np.arctan((self.d1 - self.d2)/np.sqrt((self.d1 - self.d2)**2 + self.l**2))

        # distance
        #self.md = (self.d1 + self.d2)/2

    def cb_enc(self, data):

        # signals
        s1, s2 = data.delta_encoder1, data.delta_encoder2

        # estimated velocities
        vw1, vw2 = [2*np.pi*self.r*self.f*s/self.tpr for s in (s1, s2)]

        # desired wheel velocity
        e1 = self.v - 0.115*float(self.d1 - self.d2) - vw1
        e2 = self.v + 0.115*float(self.d1 - self.d2) - vw2

        # integration error
        self.int1 += e1*0.1
        self.int2 += e2*0.1

        # control signals
        sig = PWM()
        sig.PWM1 = self.kp*e1 + self.ki*self.int1
        sig.PWM2 = self.kp*e2 + self.ki*self.int2

        #print(e1, e2)

        self.pub_pwm.publish(sig)

        print(e1, e2)





if __name__ == '__main__':

    follower(0.1, 0.5, 10, 0.1)
