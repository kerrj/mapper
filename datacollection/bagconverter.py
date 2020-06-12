import numpy as np
import rosbag
bag=rosbag.Bag('traj.bag')
ranges=[]
poses=[]
xs=[]
ys=[]
dim=0
lastodom=[0,0,0]
for topic,msg,t in bag.read_messages(topics=['/scan','/wheel_odom','/rectified_scan']):
    if topic=='/wheel_odom':
        lastodom[0]+=msg.x*np.cos(lastodom[2])
        lastodom[1]+=msg.x*np.sin(lastodom[2])
        lastodom[2]+=msg.th
    elif topic=='/scan':
        ranges.append(msg.ranges)
        poses.append(lastodom.copy())
        dim=max(dim,len(msg.ranges))
    elif topic=='/rectified_scan':
        xs.append(msg.xs)
        ys.append(msg.ys)
ranges=ranges[:-1]
bag.close()
npranges=np.zeros((len(ranges),dim))
npposes=np.array(poses)
npxs=np.zeros((len(ranges),dim))
npys=np.zeros((len(ranges),dim))
for i in range(len(ranges)):
    npranges[i,0:len(ranges[i])]=ranges[i]
    npxs[i,0:len(xs[i])]=xs[i]
    npys[i,0:len(ys[i])]=ys[i]
np.save('ranges',npranges)
np.save('poses',npposes)
np.save('xs',npxs)
np.save('ys',npys)
print("Saved output, dims: ",npranges.shape,npposes.shape,npxs.shape)
