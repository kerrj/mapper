import numpy as np
import rosbag
bag=rosbag.Bag('bagexpressnoac.bag')
ranges=[]
i=0
m=None
dim=0
for topic,msg,t in bag.read_messages(topics=['/scan']):
    ranges.append(msg.ranges)
    m=msg
    i+=1
    dim=max(dim,len(msg.ranges))
    if i==10:break
bag.close()
npranges=np.zeros((len(ranges),dim))
for i in range(len(ranges)):
    npranges[i,0:len(ranges[i])]=ranges[i]
np.save('ranges',npranges)
print(m.angle_min,m.angle_max,m.angle_increment)
print("Saved output, dims: ",npranges.shape)
