import numpy as np
import matplotlib.pyplot as plt


# Data class
class Data_class:
    def __init__(self):
        self.data = np.matrix([])
        self.dom_f = 0
        self.fit_coefficients = np.zeros((3, 1))
        self.Fs = 0
        self.N = 0
        self.time_vector = 0

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


class PredictionClass:
    def __init__(self):
        return

    def performFFT(self, predict_data):
        # perform fft and store the dominant frequency

        fft_coefficients = np.fft.fft(predict_data.data)
        fft_freq = np.fft.fftfreq(predict_data.N)

        magX = np.abs(fft_coefficients) / predict_data.N
        max_mag = np.max(magX)
        cutoff = 0.5 * max_mag
        freq_needed = [freq for val, freq in zip(np.transpose(magX), np.transpose(fft_freq)) if val > cutoff]
        predict_data.dom_f = np.mean(np.abs(freq_needed)) * predict_data.Fs  # WHAT!!!

        return

    def performFIT(self, predict_data, update):
        # perform FIT using Fourier but update depending on update variable

        t = predict_data.time_vector
        x_ = np.ones(predict_data.N)
        x_1 = [np.cos(2 * np.pi * predict_data.dom_f * t_) for t_ in t]
        x_2 = [np.sin(2 * np.pi * predict_data.dom_f * t_) for t_ in t]

        x_ = np.transpose(np.vstack((x_, x_1, x_2)))

        x_ = np.matrix(x_)
        y = np.transpose(predict_data.data)

        beta = np.linalg.inv(np.transpose(x_) * x_) * (np.transpose(x_) * y)  # leas squares solution

        if update:
            predict_data.fit_coefficients = beta

        y_hat = predict_data.calcVals(t, beta)
        sse = np.square(y_hat - y)
        sse = np.sum(sse, axis=0)
        return beta, sse

    def performFIT_SSE(self, predict_data):
        # perform

        freq_ = predict_data.dom_f
        range_freq = predict_data.dom_f / 10.0
        f_array = np.arange(freq_ - range_freq, freq_ + range_freq, freq_ / 100.0)
        min_sse = 100000000
        min_freq = 100000

        for f in f_array:
            predict_data.dom_f = f
            beta_trial, sse_trial = self.performFIT(predict_data, False)

            if sse_trial < min_sse:
                min_sse = sse_trial
                min_freq = predict_data.dom_f

        predict_data.dom_f = min_freq
        betas, minSSE = self.performFIT(predict_data, True)
        print "The Final Coefficients using SSE+Regression:\n", betas
        print "Dominant Frequency is:", predict_data.dom_f

        return betas, minSSE

    def predict_sec(self, predict_data, seconds, resolution):
        predict_time_vector = np.arange(predict_data.time_vector[-1], predict_data.time_vector[-1] + seconds,
                                        resolution)

        y_hat = np.transpose(predict_data.calcVals(predict_time_vector, predict_data.fit_coefficients))

        list_yHat = []
        for i in range(0, np.shape(y_hat)[1]):
            list_yHat.append(y_hat[0, i])

        index_min = y_hat.argmin()
        y_hat_ = y_hat[0, 0:index_min + 1]
        ind = [i for i in range(index_min - 1) if y_hat_[0, i] >= 0 and y_hat_[0, i + 1] <= 0]
        time_to_lowest = predict_time_vector[index_min]
        print ind[-1]
        time_to_move = predict_time_vector[ind[-1]]
        vel = (y_hat[0, index_min] - 0.0) / (time_to_lowest - time_to_move)

        # visualize = np.zeros(np.shape(y_hat))
        # visualize[0, index_min] = y_hat[0,index_min]
        # visualize[0, ind[-1]] = 0.0
        # list_yHat = []
        # list_vis = []
        # for i in range(0, np.shape(y_hat)[1]):
        #     list_yHat.append(y_hat[0, i])
        #     list_vis.append(visualize[0, i])
        #
        # plt.plot(predict_time_vector, list_yHat)
        # plt.plot(predict_time_vector, list_vis, 'r+')
        # plt.show()
        return time_to_lowest, time_to_move, vel, list_yHat, predict_time_vector
