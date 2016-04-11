#!/usr/bin/env python
import serial
import numpy as np
import sys
import time
import rospy
import std_msgs.msg
from dji_sdk.msg import Velocity
from palantir_pkg.msg import docked

lis=[]
m_range=20
dock_stat=0
done = 0
quad_vel=0.15

def vel_val_cb(msg):
    global quad_vel
    quad_vel=msg.vz
    print quad_vel

def detection_cb(msg):
    global dock_stat
    #print("entered detection cb")
    if msg.data == 1.0:
	dock_stat=1
	
	   
def velocity_cb(msg):
    global dock_stat
    global lis
    global m_range
    global done 
    global quad_vel
    #print("entered velocity cb")
    #print("length of list")
    l=len(lis)
    docking_pub = rospy.Publisher('/docked/rel_vel',docked,queue_size=10)
    message=docked()
    #print l
    #print("m_range")
    #print(m_range)
    #quad_vel=msg.data	

    if dock_stat==1 and l<=m_range:
	#print("if condition met")
        lis.append(msg.data)
	
    if len(lis)==m_range and done ==0:
        done = 1
    if dock_stat==1 and done == 1 and quad_vel!=0:
        platform_vel=-(lis[m_range-1]-lis[0])/(m_range*0.01*100)
        relative_vel=abs(platform_vel-quad_vel)
        print("DOCKED!!!")
        print dock_stat
        print("Platform velocity at dock=")
        print platform_vel
        print("Quadcopter velocity at dock=")
        print quad_vel
        print("Relative velocity at dock=")
        print relative_vel
        message.status="DOCKED!!"
        message.dock_status=dock_stat
        message.plat_vel=platform_vel
        message.quad_vel=quad_vel
        message.rel_vel=relative_vel
        docking_pub.publish(message) #MAKE CUSTOM MESSAGE FOR DOCKED
        done =2


def docking_velocity():
    #print("dock_stat")
    #print dock_stat
    #print("done")
    #print done
    rospy.init_node('docking_velocity', anonymous=True)
    rospy.Subscriber("/serial/dockDetection", std_msgs.msg.Float64, detection_cb)
    rospy.Subscriber("/serial/irReadings", std_msgs.msg.Float64, velocity_cb)
    rospy.Subscriber("/dji_sdk/velocity", Velocity, vel_val_cb)

    rospy.spin()

if __name__ == '__main__':
    docking_velocity()

 
