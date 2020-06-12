#!/usr/bin/env python3
import rospy
import readchar
from mapper.msg import BaseCommand
from geometry_msgs.msg import Point
rospy.init_node("teleop")
cmd_pub=rospy.Publisher("target_vel",BaseCommand,queue_size=1)
svel=.35
turnvel=1.5
print("Type command (wasd for direction). press q to exit")
rate=rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
    #cmd=input("Command: ")
    cmd=readchar.readchar()
    msg=BaseCommand()
    msg.header.stamp=rospy.get_rostime()
    msg.velocity=0
    msg.omega=0
    if 'w' in cmd:
        msg.velocity+=svel
    if 's' in cmd:
        msg.velocity-=svel
    if 'a' in cmd:
        msg.omega+=turnvel
    if 'd' in cmd:
        msg.omega-=turnvel
    if 'q' in cmd:
        exit()
    if 'l:' in cmd:
        s=cmd[cmd.find('l:')+2:]
        pairs=s.split(';')
        path=Path()
        l=[]
        for pair in pairs:
            xy=pair.split(',')
            x=float(xy[0][1:])
            y=float(xy[1][:-1])
            p=Point(x,y,0.)
            l.append(p)
        path.waypoints=l
        pathpub.publish(path)
    cmd_pub.publish(msg)

