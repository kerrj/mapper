import numpy as np
import rosbag
bag=rosbag.Bag('mapnocauchy.bag')
ranges=[]
poses=[]
xs=[]
ys=[]
mapmsg=None
dim=0
i=0
lastodom=[0,0,0]
lastotime=0
for topic,msg,t in bag.read_messages(topics=['/scan','/wheel_odom','/rectified_scan','/map']):
    if topic=='/wheel_odom':
        dt=msg.header.stamp.to_time()-lastotime
        if dt>.3:
            print("bad odom",dt,i)
        lastotime=msg.header.stamp.to_time()
        lastodom[0]+=msg.x*np.cos(lastodom[2])
        lastodom[1]+=msg.x*np.sin(lastodom[2])
        lastodom[2]+=msg.th
    elif topic=='/scan':
        i+=1
        ranges.append(msg.ranges)
        poses.append(lastodom.copy())
        dim=max(dim,len(msg.ranges))
    elif topic=='/rectified_scan':
        xs.append(msg.xs)
        ys.append(msg.ys)
    elif topic=="/map":
        mapmsg=msg
npmap=np.reshape(np.array(list(mapmsg.data)),(mapmsg.numX,mapmsg.numY))
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
np.save('map',npmap)
print("Saved output, dims: ",npranges.shape,npposes.shape,npxs.shape,npmap.shape)
