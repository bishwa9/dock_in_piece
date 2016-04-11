#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32

def callback(data):
    pub = rospy.Publisher('navigation_result', Int32, queue_size=10)
    if data.data > 1:  
        msg=0
    elif data.data < 1:
        msg=1
    pub.publish(msg)
   
def navigation():

    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber("Velocity", Float64, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    navigation()