#!/usr/bin/env python
import matplotlib.pyplot as plt
import time
import prediction_module
import serial_module
import time
import rospy
import std_msgs.msg
from palantir_pkg.msg import palantir_msg

class pubsub:
    def __init__(self,ir_name,prediction_name,state_sub_name):
        self.irPub = rospy.Publisher(ir_name, std_msgs.msg.Float64, queue_size=10)
        self.predictionPub = rospy.Publisher(prediction_name, palantir_msg, queue_size=10)
        self.st_sub = rospy.Subscriber(state_sub_name, std_msgs.msg.UInt8, self.state_cb)
        self.st = 0
        print "Palantir: Publishers and Subscribers setuped"

    def state_cb(self,state):
        self.st = state.data
        #print self.st
        return

    def publish_prediciton(self,vel, time_to_move):
        command = palantir_msg()
        command.header.stamp=rospy.Time.now()
        command.timeToMove=time_to_move
        command.velocity=vel
        self.predictionPub.publish(command)
        print "PALANTIR HAS SUBMITTED PREDICTION"
        return

    def publish_ir(self,ir_reading):
        msg = std_msgs.msg.Float64()
        msg.data = ir_reading
        self.irPub.publish(msg)
        return

if __name__ == '__main__':
    try:
        start_time = time.time()
        rospy.init_node('Palantir', anonymous=True)
    	data_obj = prediction_module.Data_class()
        prediction_obj = prediction_module.PredictionClass()
    	pubsub_obj = pubsub('/palantir/irReadings','/palantir/prediction','/state')
    	# Read from serial port
        ser = serialRead('/dev/ttyACM0', 9600)
        while (pubsub_obj.st<5):
            time.sleep(0.1)
        print 'start'
        data_read = ser.readData(1000, pubsub_obj)

    	data_obj.fill(data_read, 100.0)
        prediction_obj.performFFT(data_obj)
        blah, blah2 = prediction_obj.performFIT_SSE(data_obj)
        
        # use the learnt model to predict point
        # time_to_lowest, time_to_move, vel, pred, t = prediction_obj.predict_sec(data_obj, 10, 0.1)
        # e_time = time.time() - start_t
        # time_to_move -= e_time
        while pubsub_obj.st < 6:
            pubsub_obj.publish_prediciton(0.1, 20)
            time.sleep(0.1)
    	#data_obj.showPlot()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass