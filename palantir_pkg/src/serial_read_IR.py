#!/usr/bin/env python

import serial
import numpy as np
import sys
import time
import rospy
import std_msgs.msg


class pubsub:
    def __init__(self,ir_name,state_sub_name):
        self.irPub = rospy.Publisher(ir_name, std_msgs.msg.Float64, queue_size=100)
        self.st_sub = rospy.Subscriber(state_sub_name, std_msgs.msg.UInt8, self.state_cb)
        self.st = 0
        #print "Serial Read: Publishers and Subscribers setuped"

    def state_cb(self,state):
        self.st = state.data
        #print self.st
        return

    def publish_ir(self,ir_reading):
        msg = std_msgs.msg.Float64()
        msg.data = ir_reading
        self.irPub.publish(msg)
	#print "IR data published"
        return

class serialRead:
    def __init__(self, serialPort_, baudRate_):
        self.serialPort = serial.Serial()
        self.serialPort.port = serialPort_
        self.serialPort.baudrate = baudRate_
        self.serialPort.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.serialPort.parity = serial.PARITY_NONE  # set party check: no parity
        self.serialPort.stopbits = serial.STOPBITS_ONE  # number of stop bits
        self.serialPort.timeout = 1  # non-block read
        self.serialPort.xonxoff = False  # disable software flow control
        self.serialPort.rtscts = False  # disable hardware (RTS/CTS) flow control
        self.serialPort.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        self.serialPort.writeTimeout = 2  # timeout for write

    def readData(self):
	data_e=10000.0
        try:
            data_e = self.serialPort.readline()
            data_e = data_e.rstrip()
	except Exception, e:
            print "Error reading data:", str(e)     
        return data_e



if __name__ == '__main__':
    try:
    	time.time()
        rospy.init_node('serial_read_IR', anonymous=True)
    	pubsub_obj = pubsub('/serial/irReadings','/state')
    	# Read from serial port
        ser2 = serialRead('/dev/ttyACM0', 9600) # ACM1
        rate = rospy.Rate(100)     #10 = 10Hz CHANGE LATER
        print "Serial_read: Waiting to read IR"
	time.sleep(5)
	ser2.serialPort.open() 
	if ser2.serialPort.isOpen():
            ser2.serialPort.flushInput()  # flush input buffer, discarding all its contents
            ser2.serialPort.flushOutput() 
	while not rospy.is_shutdown():    		
            data_IR=ser2.readData()
	    if data_IR.isdigit() == True:
		data_IR=float(data_IR)
	    	pubsub_obj.publish_ir(data_IR)   

	    rate.sleep()
    except rospy.ROSInterruptException:
	pass
    rospy.spin()
