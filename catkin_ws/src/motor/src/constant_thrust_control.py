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
            self.thrust_constant = rospy.get_param('~thrust_constant', 50)
            self.cutoff  = rospy.get_param('~cutoff_value', 20)

            # Initialize
            self.motor = Motor()
            self.thrust = 0.0
            self.rudder = 0.0
            self.L = 0.0
            self.R = 0.0
            self.output_L = 0.0
            self.output_R = 0.0


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

            self.thrustRudderToLR()
            rospy.loginfo("Thrust by moos L :  %f ", self.L)
            rospy.loginfo("Thrust by moos R :  %f ", self.R)

            # Thrust to constant
            self.output_L = self.constant_thrust(self.L/100.0)
            self.output_R = self.constant_thrust(self.R/100.0)
            rospy.loginfo("Constant thrust L :  %f ", self.output_L)
            rospy.loginfo("Constant thrust R :  %f ", self.output_R)

            # Publish data
            self.motor.header = Header(frame_id=THRUST.FRAME_ID, stamp=rospy.Time.now())
            self.motor.left = self.output_L  #* 0.01
            self.motor.right = self.output_R #* 0.01
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

        def constant_thrust(self, value): # /100.0
            sign = 1.0 if value > 0.0 else -1.0
            tmp = math.fabs(value)
            stop_value = self.cutoff * 0.01
            if tmp < stop_value:
                output = 0.0
            else:
                output = sign * self.thrust_constant * 0.01
            return output


        def onShutdown(self):
            rospy.loginfo("Thrust_control node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('thrust_control')
	node = THRUST()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
