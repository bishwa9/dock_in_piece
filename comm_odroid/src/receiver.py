#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Int32
from palantir_pkg.msg import palantir_msg
import time
import sys

decision_pub = rospy.Publisher('/decision/prediction', Float64, queue_size=10)
debug_pub_start = rospy.Publisher('/debug/waiting_start', UInt8, queue_size=10)
debug_pub_stop = rospy.Publisher('/debug/waiting_stop', UInt8, queue_size=10)
debug1 = 10
debug2 = 10
cur_st = 0
palantir_decision_made_st = 6

def timer_cb(event):
	debug_pub_start.publish(debug1)
	debug_pub_stop.publish(debug2)
	return

def st_cb(st):
	global cur_st
	cur_st = st.data
	return

def callback(data):
	global debug1
	global debug2
	global cur_st

	#print "Decision_node state", cur_st
	if cur_st == palantir_decision_made_st:

		if data.timeToMove < 0:
			print "WILL NOT DOCK!\nFREQUENCY TOO HIGH!\nLanding!"
			velocity=data.velocity
			decision_pub.publish(velocity)
			return

		time_start=rospy.Time.now()
		print "time start receiver", time_start		
		timeToMove=data.timeToMove-1.5
		print "Move in", timeToMove
		velocity=data.velocity
		debug1 = 100
		time_ = time.time()
		while rospy.Time.now() < time_start + rospy.Duration.from_sec(timeToMove):
			print "Decision_node waiting", time.time() - time_
			time.sleep(0.2)
		print "time done receiver", rospy.Time.now()
		print "time waited for receiver", rospy.Time.now() - time_start
		debug2 = 100
		decision_pub.publish(velocity)
    
def receiver():
    rospy.init_node('Decision_node', anonymous=True)
    rospy.Subscriber("/palantir/prediction", palantir_msg, callback)
    rospy.Subscriber("/state", UInt8, st_cb)
    rospy.Timer(rospy.Duration(0.2), timer_cb)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	receiver()