#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.interpolate import interp1d
from sensor_msgs.msg import LaserScan
from mapper.msg import Odometry,RectifiedScan
SEAM_OFFSET=-8
'''
Combines wheel odometry and scan streams and outputs a rectified scan
for each which compensates for warping due to movement. Output is
a laser scan in (x,y) coords in the robot frame at the beginning of the scan.
In other words, the coordinate frame is where the robot was when the scan 
started.
'''
class Rectifier:
    def __init__(self,pub=None):
        self.odomQ=[(0,0,0)]
        self.lastScanTime=None
        self.pub=pub
    def scanCB(self,msg):
        print("start scan")
        if self.lastScanTime is None:
            self.lastScanTime=msg.header.stamp
            print("end scan")
            return
        self.lastScanTime=msg.header.stamp
        #we only publish the rectified scan if the data is good (we are clearly interpolating between 2 scans)
        #do some rectifying shit
        rectscan=RectifiedScan()
        rectscan.header.frame_id='wheel_base'
        rectscan.header.stamp=msg.header.stamp
        rectscan.scan=msg
        xs,ys=self.rectify(msg)
        #dump the odomQ and set last scan
        rectscan.xs=xs
        rectscan.ys=ys
        self.odomQ=[(0,0,0)]
        self.pub.publish(rectscan)
        print("end scan")
    def odomCB(self,msg):
        print("Start odom")
        if self.lastScanTime is None:
            print("end odom")
            return
        if msg.header.stamp<self.lastScanTime:
            print("end odom")
            return#ignore stale data
        prevO=self.odomQ[-1]
        self.odomQ.append((prevO[0]+msg.x,prevO[1]+msg.y,prevO[2]+msg.th))
        print("end odom")
    def rectify(self,scan):
        #handles the process of transforming the laserscan given
        '''
        IMPORTANT this only works in boost+angle correct mode since it assumes 720 data points
        per scan. 
        '''
        assert(len(scan.ranges)==720)#catch if i accidentally break this later lol
        poses=np.array(self.odomQ)
        angles=np.linspace(0,719,poses.shape[0])
        interpF=interp1d(angles,poses,kind='linear',axis=0,assume_sorted=True)
        xs=np.zeros(720)
        ys=np.zeros(720)
        for i in range(len(scan.ranges)):
            th=np.radians(.5*i)
            d=scan.ranges[i]
            t=interpF((i+2*SEAM_OFFSET)%720)
            R=np.array([[np.cos(t[2]),-np.sin(t[2])],[np.sin(t[2]),np.cos(t[2])]])
            datap=np.array([[d*np.cos(th)],[d*np.sin(th)]])
            trans=R.dot(datap)+np.array([[t[0]],[t[1]]])
            xs[i]=trans[0,0]
            ys[i]=trans[1,0]
        return xs,ys
if __name__=="__main__":
    rospy.init_node("scan_rectifying")
    pub=rospy.Publisher('/rectified_scan',RectifiedScan,queue_size=10)
    r=Rectifier(pub)
    rospy.Subscriber('/scan',LaserScan,r.scanCB,queue_size=10)
    rospy.Subscriber('/wheel_odom',Odometry,r.odomCB,queue_size=10)
    rospy.loginfo("Rectifier node running")
    rospy.spin()
