#!/usr/bin/env python

import rospy
import math
import numpy as np
from motor.msg import Motor
from std_msgs.msg import Header
from std_msgs.msg import Float32


class THRUST:

        # Static params
        FRAME_ID = "base_link"


	def __init__(self):

            # OnstartUp
            self.max_thrust_limit = rospy.get_param('~max_thrust_limit', 100)
            self.min_thrust_limit = rospy.get_param('~min_thrust_limit', 50)

            # Initialize
            self.motor = Motor()
            self.thrust = 0.0
            self.rudder = 0.0
            self.L = 0.0
            self.R = 0.0
            self.output_L = 0.0
            self.output_R = 0.0
            self.delta = (self.max_thrust_limit - self.min_thrust_limit) / 100.0


            # Subscriber
            rospy.Subscriber("DESIRED_THRUST", Float32, self.thrust_cb)
            rospy.Subscriber("DESIRED_RUDDER", Float32, self.rudder_cb)

            # Publisher
	    self.pub = rospy.Publisher("motor", Motor, queue_size=10)

        def thrust_cb(self, msg):

            # Subcribe data
            self.thrust = msg.data

           
        def rudder_cb(self, msg):

            # Subcribe data
            self.rudder = msg.data
            #print"Thust : ", self.thrust
            #print"Rudder : ", self.rudder

            self.thrustRudderToLR()
            #print"Thrust L : ", self.L
            #print"Thrust R : ", self.R
            self.output_L = self.valid_range(self.L)/ 100.0
            self.output_R = self.valid_range(self.R)/ 100.0
            #print"Thrust output L : ", self.output_L
            #print"Thrust output R : ", self.output_R


            # Publish data
            self.motor.header = Header(frame_id=THRUST.FRAME_ID, stamp=rospy.Time.now())
            self.motor.left = self.output_L
            self.motor.right = self.output_R
            self.pub.publish(self.motor)

        def thrustRudderToLR(self):
            # 1. Constrain Values
            desiredRudder = self.clamp (self.rudder, (-1.0 * 100), 100.0)
            desiredThrust = self.clamp (self.thrust, (-1.0 * 100), 100.0)
            
            # 2. Calculate turn
            percentLeft  = desiredThrust + desiredRudder
            percentRight = desiredThrust - desiredRudder
            
            # 3. Map desired thrust values to motor bounds
            fwdOrRevL   = 1.0 if percentLeft  > 0.0 else -1.0
            fwdOrRevR   = 1.0 if percentRight  > 0.0 else -1.0
            pctThrustL  = math.fabs(percentLeft)  / 100.0
            pctThrustR  = math.fabs(percentRight) / 100.0
            mappedLeft  = pctThrustL * 100.0 * fwdOrRevL
            mappedRight = pctThrustR * 100.0 * fwdOrRevR
            
            # 4. Deal with overages
            maxThrustNeg = -1.0 * 100.0
            if (mappedLeft  > 100.0):
              mappedRight -= (mappedLeft  - 100.0)
            if (mappedLeft  < maxThrustNeg):
              mappedRight -= (mappedLeft  + 100.0)
            if (mappedRight > 100.0):
              mappedLeft  -= (mappedRight - 100.0)
            if (mappedRight < maxThrustNeg):
              mappedLeft  -= (mappedRight + 100.0)
            
            self.L  = self.clamp(mappedLeft,  (-1.0 * 100.0), 100.0)
            self.R  = self.clamp(mappedRight, (-1.0 * 100.0), 100.0)

        def clamp(self, v, minv, maxv):
            return min(maxv,max(minv, v))

        def valid_range(self, vel):
            sign = 1.0 if vel > 0.0 else -1.0
            tmp = math.fabs(vel)
            value = self.min_thrust_limit + self.delta * tmp
            output = sign * value
            return output

        def onShutdown(self):
            rospy.loginfo("Thrust_control node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('thrust_control')
	node = THRUST()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
