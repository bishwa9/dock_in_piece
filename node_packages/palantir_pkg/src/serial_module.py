import serial

class serialRead:
    def __init__(self, serialPort_, baudRate_):
        self.serialPort = serial.Serial()
        self.serialPort.port = serialPort_
        self.serialPort.baudrate = baudRate_
        self.serialPort.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.serialPort.parity = serial.PARITY_NONE  # set parity check: no parity
        self.serialPort.stopbits = serial.STOPBITS_ONE  # number of stop bits
        self.serialPort.timeout = 1  # non-block read
        self.serialPort.xonxoff = False  # disable software flow control
        self.serialPort.rtscts = False  # disable hardware (RTS/CTS) flow control
        self.serialPort.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        self.serialPort.writeTimeout = 2  # timeout for write

    def readData(self, num):
        data_ = []
        try:
            self.serialPort.open()
            if self.serialPort.isOpen():
                self.serialPort.flushInput()  # flush input buffer, discarding all its contents
                self.serialPort.flushOutput()  # flush output buffer, aborting current output
                #print "Waiting for 5 seconds before starting collection"
                numOfLines = 0
                #time.sleep(5)
                while (numOfLines < num):
                    data_e = self.serialPort.readline()
                    data_e = data_e.rstrip()
                    if data_e.isdigit() == False:
                        continue
                    data_e = float(data_e)
                    print numOfLines, data_e
                    numOfLines += 1
                    data_.append(data_e)
            self.serialPort.close()
        except Exception, e:
            print "Error reading data:", str(e)
        return data_
