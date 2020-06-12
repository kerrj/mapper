#!/usr/bin/env python3
import rospy
from mapper.msg import EncoderReading,Odometry
import numpy as np
'''
Publishes intermediate differentials in odometry based on wheel movement.
Odometry is always relative to the previous odometry update (dropping updates
will cause loss of information). Odometry is done this way to better allow for
integration with slam, since relative poses are needed.
Summing all messages on this topic will result in the original final odom
estimate due to wheel integration.
'''
WHEEL_RAD=.04
WHEEL_SEP=.1978
rospy.init_node('wheel_odom')
pub=rospy.Publisher('wheel_odom',Odometry,queue_size=10)
odom=Odometry()
odom.x=0;odom.y=0;odom.th=0
def enc_cb(encMsg):
    #RK4 stuff here for comparison
    l_d=encMsg.rawLeftDelta*WHEEL_RAD
    r_d=encMsg.rawRightDelta*WHEEL_RAD
    vd=(l_d+r_d)/2.0
    dth=(r_d-l_d)/WHEEL_SEP
    RT=odom.th
    opt1=RT+dth
    opt2=vd/6.0
    opt3=dth/2.0
    opt4=RT+opt3
    omsg=Odometry()
    omsg.header.stamp=encMsg.header.stamp
    omsg.header.frame_id="origin"
    dx=(opt2)*(np.cos(RT)+4.0*np.cos(opt4)+np.cos(opt1))
    dy=(opt2)*(np.sin(RT)+4.0*np.sin(opt4)+np.sin(opt1))
    th=opt1
    omsg.th=th-odom.th
    omsg.x=dx
    omsg.y=dy
    odom.x+=dx
    odom.y+=dy
    odom.th=th
    pub.publish(omsg)
rospy.Subscriber('encoders',EncoderReading,enc_cb,queue_size=10)
rospy.spin()
