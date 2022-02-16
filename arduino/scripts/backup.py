#!/usr/bin/env python3

import matplotlib

from sensor_msgs.msg import PointCloud
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import math
import numpy as np
import matplotlib.animation as anim
import time
myvar = None
x = []
y = []
color = []
frame = [[0],[0],[0]]
value = []
f = 0
update = 0
a = 0
b= 0
k = 0
#scat=plt.scatter(x,y,c=color,s=0.8,marker='.',cmap= plt.cm.get_cmap('RdYlBu'))

def distance_callback(data):

   # print(data.channels[0].values[50])  
        #print(data.points[].x[:])
       #print(data.points[:].y)      
   # else:
    
  #  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    
    def update(data):
        global i,tim,k,scat,fig,ax,x,y,color

        tim = time.time()
        for i in range(399):
            y.append(data.points[i].y)
            x.append(data.points[i].x)
            color.append(data.channels[0].values[i])
        frame = [x,y,color]
        print(k)
        if k ==0 :
            print(i)
            
            
            fig,ax = plt.subplots()
            ax.set_ylim([0,100])
            ax.set_xlim([0,100])
            scat=ax.scatter(x,y,c=color,s=0.8,marker='.',cmap= plt.cm.get_cmap('RdYlBu'))
            k +=1
        else:    
            scat.set_offsets(np.array([x,y]).T)
            scat.set_array(np.array(color))
            
        
        plt.draw()
        plt.pause(0.00001)
        print(tim - time.time())
        return scat
    update(data)
    




def listener():
    rospy.init_node("plot_serial", anonymous = True)
    rospy.Subscriber("/tritech_micron/scan",PointCloud, distance_callback)
    #angle = rospy.Subscriber("Angle",Float64,angle_callback)
    rospy.spin()
    plt.show()
   # print(angle)
        # plt.polar()
   # plt.scatter(angle,distance)
   # plt.show()

if __name__ == '__main__':
   # plt.xlim([-100, 100])
   # plt.ylim([-100,100])
    listener() 
