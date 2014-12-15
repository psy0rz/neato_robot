#!/usr/bin/env python

# ROS node for the Neato Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
ROS node for Neato XV-11 Robot Vacuum.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from neato_driver.neato_driver import xv11, BASE_WIDTH, MAX_SPEED

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        self.port = rospy.get_param('~port', "/dev/ttyUSB0")
        rospy.loginfo("Using port: %s"%(self.port))

        self.robot = xv11(self.port)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.scanPub = rospy.Publisher('base_scan', LaserScan, queue_size=10)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_vel = [0,0] 

    def spin(self):        
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        prev_odom_stamp = rospy.Time.now()

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link)) 
        scan.angle_min = 0
        scan.angle_max = 6.26
        scan.angle_increment = 0.017437326
        scan.range_min = 0.020
        scan.range_max = 5.0
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
    
        prev_cmd_vel=[]

        try:
            # main loop of driver
            while not rospy.is_shutdown():

                # send updated movement commands, if changed otherwise every second
                if self.cmd_vel!=prev_cmd_vel or (rospy.Time.now()-last_move).to_sec()>=(1-0.2):
                    self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1], max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
                    prev_cmd_vel=self.cmd_vel[:]
                    last_move=rospy.Time.now()

                #laser scanner
                scan.header.stamp = rospy.Time.now() #try to get this as close to the actual time of the scan as possible.
                self.robot.requestScan()
                scan.ranges = self.robot.getScanRanges()
                self.scanPub.publish(scan)

                # get motor encoder values
                odom_stamp=rospy.Time.now()
                left, right = self.robot.getMotors()
                dt = (odom_stamp - prev_odom_stamp).to_sec()
                prev_odom_stamp=odom_stamp
                print(dt)

                #calculate delta-S of each wheel in meters
                d_left = (left - encoders[0])/1000.0
                d_right = (right - encoders[1])/1000.0
                encoders = [left, right]
                
                # distance traveled is the average of the two wheels
                dx = (d_left+d_right)/2
                # this approximation works (in radians) for small angles
                dth = (d_right-d_left)/(BASE_WIDTH/1000.0)

                x = cos(dth)*dx
                y = -sin(dth)*dx
                self.x += cos(self.th)*x - sin(self.th)*y
                self.y += sin(self.th)*x + cos(self.th)*y
                self.th += dth

                #komt tot zo ver overeen met https://code.google.com/p/differential-drive/source/browse/nodes/diff_tf.py

                # prepare tf from base_link to odom
                quaternion = Quaternion()
                quaternion.z = sin(self.th/2.0)
                quaternion.w = cos(self.th/2.0)

                # prepare odometry
                odom.header.stamp = odom_stamp
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0
                odom.pose.pose.orientation = quaternion
                #NOTE: differential drive robots can only move in the x-axis and rotate in along the z-axis            
                odom.twist.twist.linear.x = dx/dt
                odom.twist.twist.angular.z = dth/dt

                # publish everything
                self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    odom_stamp, "base_link", "odom" )
                self.odomPub.publish(odom)

                # wait, then do it again
                #dont sleep, its slow enough already. 
                #r.sleep()
        except:
            # always try to shut down
            self.robot.setLDS("off")
            self.robot.setTestMode("off") 
            raise

        # shut down
        self.robot.setLDS("off")
        self.robot.setTestMode("off") 

    def cmdVelCb(self,req):

        if req.linear.y!=0 or req.linear.z!=0:
            rospy.logerr("Warning, only linear movement in x direction supported")

        if req.angular.x!=0 or req.angular.y!=0:
            rospy.logerr("Warning, only angular movement in z direction is supported")

        x = req.linear.x * 1000
        th = req.angular.z * (BASE_WIDTH/2) 
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k
        self.cmd_vel = [ int(x-th) , int(x+th) ]

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()

