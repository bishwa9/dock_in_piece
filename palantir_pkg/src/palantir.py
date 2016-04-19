#!/usr/bin/env python
import numpy as np
import sys
import signal
import matplotlib.pyplot as plt
import time
import rospy
import std_msgs.msg
from palantir_pkg.msg import palantir_msg
# from std_msgs import UInt8

def signal_handler(signal,frame):
    print('You killed palatir successfully')
    sys.exit(0)
# Data class
class Data_class:
    def __init__(self):
        self.data = np.matrix([])
        self.dom_f = 0
        self.fit_coefficients = np.zeros((3, 1))
        self.Fs = 0
        self.N = 0
        self.time_vector = 0
        self.safety_time = 8

    def fill(self, data_, Fs_):
        self.data = np.matrix(data_ - np.mean(data_))
        self.Fs = Fs_
        self.N = len(data_)
        self.time_vector = np.arange(0, self.N, 1) / self.Fs
        return

    def showPlot(self):
        list_y = []
        list_yHat = []
        y_hat = np.transpose(self.calcVals(self.time_vector, self.fit_coefficients))
        for i in range(0, np.shape(self.data)[1]):
            list_y.append(self.data[0, i])
            list_yHat.append(y_hat[0, i])
        t = self.time_vector

        plt.plot(t, list_y, 'r', label='IR_Data')
        plt.plot(t, list_yHat, 'b', label='Prediction')
        legend = plt.legend(loc='upper center', shadow=True)
        plt.title("Prediction on 2Hz")
        plt.show()
        return

    def calcVals(self, timeVector, beta):
        y_hat = [beta[0, 0] + beta[1, 0] * np.cos((2 * np.pi) * self.dom_f * t_) + beta[2, 0] * np.sin(
            (2 * np.pi) * self.dom_f * t_)
                 for t_ in timeVector]
        y_hat = np.transpose(np.matrix(y_hat))
        return y_hat

    def performFFT(self):
        # perform fft and store the dominant frequency

        fft_coefficients = np.fft.fft(self.data)
        fft_freq = np.fft.fftfreq(self.N)

        magX = np.abs(fft_coefficients) / self.N
        max_mag = np.max(magX)
        cutoff = 0.5 * max_mag
        freq_needed = [freq for val, freq in zip(np.transpose(magX), np.transpose(fft_freq)) if val > cutoff]
        self.dom_f = np.mean(np.abs(freq_needed)) * self.Fs  # WHAT!!!

        return

    def performFIT(self, update):
        # perform FIT using Fourier but update depending on update variable

        t = self.time_vector
        x_ = np.ones(self.N)
        x_1 = [np.cos(2 * np.pi * self.dom_f * t_) for t_ in t]
        x_2 = [np.sin(2 * np.pi * self.dom_f * t_) for t_ in t]

        x_ = np.transpose(np.vstack((x_, x_1, x_2)))

        x_ = np.matrix(x_)
        y = np.transpose(self.data)

        beta = np.linalg.inv(np.transpose(x_) * x_) * (np.transpose(x_) * y)  # leas squares solution

        if update:
            self.fit_coefficients = beta

        y_hat = self.calcVals(t, beta)
        sse = np.square(y_hat - y)
        sse = np.sum(sse, axis=0)
        return beta, sse

    def performFIT_SSE(self):
        # perform

        freq_ = self.dom_f
        range_freq = self.dom_f / 10.0
        f_array = np.arange(freq_ - range_freq, freq_ + range_freq, freq_ / 100.0)
        min_sse = 100000000
        min_freq = 100000

        for f in f_array:
            self.dom_f = f
            beta_trial, sse_trial = self.performFIT(False)

            if sse_trial < min_sse:
                min_sse = sse_trial
                min_freq = self.dom_f

        self.dom_f = min_freq
        betas, minSSE = self.performFIT(True)
        print "Palantir: The Final Coefficients using SSE+Regression:\n", betas
        print "Palantir: Dominant Frequency is:", self.dom_f

        return betas, minSSE

    def predict_sec(self, seconds, resolution, timeToRead):
        
        predict_time_vector = np.arange(self.time_vector[-1], self.time_vector[-1]+seconds, resolution)

        y_hat = np.transpose(self.calcVals(predict_time_vector, self.fit_coefficients))

        ind_1 = int(1/(self.dom_f*resolution))
        ind_2 = int(2/(self.dom_f*resolution))
        print "time_ind1",predict_time_vector[ind_1]
        print "val_ind1", y_hat[0, ind_1]
        print "time_ind2",predict_time_vector[ind_2]
        print "val_ind2", y_hat[0, ind_2]
        print "time_rand1", predict_time_vector[int( (ind_1+ind_2)/2.0 )]
        print "val_rand1", y_hat[0, int( (ind_1+ind_2)/2.0 )]
        print "time_rand2", predict_time_vector[int(ind_1+100)]
        print "val_rand2", y_hat[0, int( ind_1+100 )]
        
        # print "Time predicting for:", predict_time_vector[-1]
        # print "ind_1", ind_1
        # print "ind_2", ind_2
        #y_hat_1 = y_hat[0,0:ind_1]
        y_hat_2 = y_hat[0,ind_1:ind_2]
        #y_hat_3 = y_hat[0,ind_2::]
        index_min = y_hat_2.argmax() + ind_1
        print "Index min:", index_min
        print "time_index_min: ", predict_time_vector[index_min]

        #y_hat = y_hat[0,index_min+1::]

        # index_min = y_hat.argmin()
        # y_hat_ = y_hat_2[0,0:index_min+1]
        ind = [i for i in range(index_min-1) if y_hat[0,index_min-i] >= 0 and y_hat[0,index_min-i-1] < 0]
        print "ind", ind
        time_to_lowest = predict_time_vector[index_min]
        time_to_move = predict_time_vector[index_min - ind[0]]
        vel = 60.0 / (time_to_lowest - time_to_move)
        vel = vel/100.0 #convert to m/s
        # print "predicted", y_hat

        return time_to_lowest, time_to_move, vel

class pubsub:
    def __init__(self,prediction_name,state_sub_name,ir_sub_name,ir_num):
        self.predictionPub = rospy.Publisher(prediction_name, palantir_msg, queue_size=10, latch=True)
        self.st_sub = rospy.Subscriber(state_sub_name, std_msgs.msg.UInt8, self.state_cb)
        self.ir_sub = rospy.Subscriber(ir_sub_name, std_msgs.msg.Float64, self.ir_cb)
        self.debug_start_pub = rospy.Publisher("/debug/collection_start", std_msgs.msg.UInt8, queue_size=10, latch=True)
        self.debug_stop_pub = rospy.Publisher("/debug/collection_stop", std_msgs.msg.UInt8, queue_size=10, latch=True)
        self.debug_rise_pub = rospy.Publisher("/debug/palantir_rise", std_msgs.msg.UInt8, queue_size=10, latch=True)
        self.debug_rise_pub = rospy.Publisher("/debug/palantir_rise", std_msgs.msg.UInt8, queue_size=10, latch=True)
        rospy.Timer(rospy.Duration(0.2), self.timer_cb)
        self.st = 0
        self.data_read = ir_num*[0]
        self.num_ir_read = 0
        self.num_to_read = ir_num
        self.start_time  = time.time()
        self.time_took_to_read = time.time()
        self.printed = False
        self.started = 10
        self.stopped = 10
        self.rise = 10
        # print "Palantir: Publishers and Subscribers setuped"

    def timer_cb(self, event):
        self.debug_start_pub.publish(self.started)
        self.debug_stop_pub.publish(self.stopped)
        self.debug_rise_pub.publish(self.rise)
        return

    def state_cb(self,state):
        self.st = state.data
        # print self.st
        return

    def publish_prediciton(self,vel, time_to_move):
        command = palantir_msg()
        command.header.stamp=rospy.Time.now()
        command.timeToMove=time_to_move
        command.velocity=vel
        self.predictionPub.publish(command)
        # print "PALANTIR HAS SUBMITTED PREDICTION"
        return

    def ir_cb(self,ir_reading):
        if self.num_ir_read == 0:
            self.start_time = time.time()
            print "time started data collection", self.start_time

        if self.num_ir_read == self.num_to_read and not self.printed:
            self.stopped = 100   
            self.time_took_to_read = time.time() - self.start_time 
            print "IR Reading Done Time:", time.time()
            print "Data collected for:", self.time_took_to_read
            self.printed = True
            # file_ = open("~/collected.txt", "w")
            # for i in self.data_read:
            #     file_.write(str(i)+"\n")  

        if(self.num_ir_read < self.num_to_read and self.st >= 5):
            self.started = 100
            self.data_read[self.num_ir_read] = ir_reading.data
            self.num_ir_read = 1 + self.num_ir_read
        return

    def return_data(self):
        return self.data_read[:]



if __name__ == '__main__':
    try:
        
        rospy.init_node('Palantir', anonymous=True)
    	data_obj = Data_class()
    	pubsub_obj = pubsub('/palantir/prediction', '/state', '/serial/irReadings', 1000)
        # debug_pub = rospy.Publisher('/debug/palantir', UInt8, queue_size=10)
        signal.signal(signal.SIGINT, signal_handler)
        while (pubsub_obj.num_ir_read < pubsub_obj.num_to_read):
            time.sleep(0.2)
        data_read_ = pubsub_obj.return_data()
        print "Time Processing starts", time.time()
    	data_obj.fill(data_read_, 50.0)
        #print "prediction elapsed time", data_obj.time_vector[-1]
    	data_obj.performFFT()
    	blah, blah2 = data_obj.performFIT_SSE()
        num_cycles = 3.0
        print "Time Prediction starts", time.time()
        time_to_lowest, time_to_move, vel = data_obj.predict_sec(num_cycles/data_obj.dom_f, 0.01, pubsub_obj.time_took_to_read)
        elapsed_time = time.time() - pubsub_obj.start_time
        time_to_move -= elapsed_time
        time_to_lowest -= elapsed_time
        print "Prediction done", time.time()
        print "elapsed time", elapsed_time
        print "time to move", time_to_move
        print "time to lowest", time_to_lowest
        print "Time vector", data_obj.time_vector[0], data_obj.time_vector[-1]
        # print "done"
        if data_obj.dom_f > 0.16:
            time_to_move = -10
            vel = -0.3
        while pubsub_obj.st < 6:
            pubsub_obj.publish_prediciton(vel, time_to_move)
            time.sleep(1)
        pubsub_obj.publish_prediciton(vel, time_to_move)
        time_wait = time.time()
        while time.time() < time_wait + time_to_move:
            print "Palantir wait:",time.time() - time_wait
            time.sleep(0.2)        
        pubsub_obj.rise = 100
    	#data_obj.showPlot()
        rospy.spin()

    except rospy.ROSInterruptException:
        print "Error"
