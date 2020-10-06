import serial
import time

class HeadMotion:
    def __init__(self, com, baud):
        self.str_comport = com
        self.str_baudrate = baud

    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.str_baudrate, timeout=0)

    def sit(self):
        package = [255,255,1,4,3,2,51,194]
        self.serialDevice.write(package)
    