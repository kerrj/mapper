#!/usr/bin/env python3
import rospy
import numpy as np
from mapper.msg import BaseCommand,MotorCommand
from util import clip,increment
WHEEL_RAD=.05
WHEEL_SEP=.1978
RATE=30
MAX_LINEAR_VEL=.5
COMMAND_TIMEOUT=.4
MAX_ANG_VEL=4
WHEEL_ACC=15*(1/RATE)
targetVel=BaseCommand()
command=MotorCommand()
def targetCB(msg):
    global targetVel,ok_to_sleep
    if abs(msg.velocity)>MAX_LINEAR_VEL or abs(msg.omega)>MAX_ANG_VEL:
        scale=max(abs(msg.velocity)/MAX_LINEAR_VEL,abs(msg.omega)/MAX_ANG_VEL)
        msg.velocity/=scale
        msg.omega/=scale
    targetVel=msg
rospy.init_node("base_controller")
targetSub=rospy.Subscriber('/target_vel',BaseCommand,targetCB,queue_size=1)
motorPub=rospy.Publisher('/cmd_vel',MotorCommand,queue_size=1)
rate=rospy.Rate(RATE)
while not rospy.is_shutdown():
    rate.sleep()
    t=rospy.get_rostime()
    if t-targetVel.header.stamp>rospy.Duration(COMMAND_TIMEOUT):
        targetVel.velocity=0
        targetVel.omega=0
        targetVel.header.stamp=t
    l_target=targetVel.velocity-(targetVel.omega*WHEEL_SEP)/2
    r_target=targetVel.velocity+(targetVel.omega*WHEEL_SEP)/2
    command.header.stamp=t
    command.left,dl=increment(command.left,l_target/WHEEL_RAD,WHEEL_ACC)
    command.right,dr=increment(command.right,r_target/WHEEL_RAD,WHEEL_ACC)
    motorPub.publish(command)
