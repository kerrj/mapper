import rosbag
import numpy as np
import imageio
bag=rosbag.Bag('croptest.bag')
size=0
msglist=list(bag.read_messages(topics=['/map']))
#size=max([max(msglist[i][1].map.numX,msglist[i][1].map.numY) for i in range(len(msglist))])
size=max([max(msglist[i][1].numX,msglist[i][1].numY) for i in range(len(msglist))])
frame=np.zeros((size,size),dtype='uint8')
with imageio.get_writer('map.gif',mode='I') as writer:
    for _,msg,_ in msglist: 
        #sx=msg.map.numX
        #sy=msg.map.numY
        sx=msg.numX
        sy=msg.numY
        startx=size//2-sx//2
        starty=size//2-sy//2
        #maparr=np.frombuffer(msg.map.data,dtype='uint8')
        maparr=np.frombuffer(msg.data,dtype='uint8')
        maparr=maparr.reshape((sx,sy))
        frame[startx:startx+sx,starty:starty+sy]=maparr
        writer.append_data(frame)
        print("wrote frame")
bag.close()
