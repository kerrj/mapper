import numpy as np
import rosbag
bag=rosbag.Bag('traj_7_straight_hall.bag')
ranges=[]
poses=[]
xs=[]
ys=[]
dim=0
lastodom=[0,0,0]
for topic,msg,t in bag.read_messages(topics=['/scan','/wheel_odom','/rectified_scan']):
    if topic=='/wheel_odom':
        lastodom[0]+=msg.x
        lastodom[1]+=msg.y
        lastodom[2]+=msg.th
    elif topic=='/scan':
        ranges.append(msg.ranges)
        poses.append(lastodom.copy())
        dim=max(dim,len(msg.ranges))
    elif topic=='/rectified_scan':
        xs.append(msg.xs)
        ys.append(msg.ys)
bag.close()
npranges=np.zeros((len(ranges),dim))
npposes=np.array(poses)
npxs=np.array(xs)
npys=np.array(ys)
for i in range(len(ranges)):
    npranges[i,0:len(ranges[i])]=ranges[i]
np.save('ranges',npranges)
np.save('poses',npposes)
np.save('xs',npxs)
np.save('ys',npys)
print("Saved output, dims: ",npranges.shape,npposes.shape,npxs.shape)
