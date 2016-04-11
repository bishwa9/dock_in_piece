#!/usr/bin/env python

import rospy
#from std_msgs.msg import Float64
from comm_odroid.msg import palantir_data

def transmitter():
    pub = rospy.Publisher('laptop_transmit', palantir_data, queue_size=10)
    rospy.init_node('transmitter', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    command = palantir_data()
    command.header.stamp=rospy.Time.now()
    print rospy.Time.now()
    command.timeToMove=30
    command.velocity=3
    while not rospy.is_shutdown():
    	#print "hey"
    	pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        transmitter()
    except rospy.ROSInterruptException:
        pass
