import serial
import time
import threading

class Locomotion:
    def __init__(self, com, baud, robot_state):
        self.str_comport = com
        self.baudrate = baud
        self.robot_state = robot_state
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("Initial Locomotion : comport = " + self.str_comport + " , baudrate = " + str(self.baudrate))
        try:
            self.connect()
            print("Locomotion initial successfully + + +")
        except Exception as e:
            print(e)
            print("Locomotion initial fail!!")

    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.baudrate, timeout=0)

    
    def open_run_locomotion_process(self):
        self.locomotion_process = threading.Thread(target=self.run_locomotion_process,args=())

        self.locomotion_process.start()
    def close_run_locomotion_process(self):
        self.locomotion_process.joint()

    def run_locomotion_process(self):
        while True:
            self.read_standing_status()

            if(self.robot_state[1][3] != None):
                self.set_locomotion(self.robot_state[1][3])
                


    def read_standing_status(self):
        responsePacket_length = 7
        package = [255,255,1,4,2,3,1,244]
        self.serialDevice.write(package)
        time.sleep(0.01)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())
        #print("responsePacket status=", responsePacket)
        #print("responsePacket length=", len(responsePacket))
        if(len(responsePacket) == responsePacket_length):
            # if(responsePacket[5] == 0):
            #     print("robot standing")
            # else:
            #     print("robot falling " + str(responsePacket[5]))
            ##### return to server #####
            self.robot_state[0] = responsePacket[5]
        else:
            self.robot_state[0] = None
    


    def set_locomotion(self, locomotion_command):

            if(locomotion_command == "stop"):
                self.stop_walk()
            elif(locomotion_command == "forward"):
                self.forward_walk()
            elif(locomotion_command == "turn_left"):
                self.turn_left()
            elif(locomotion_command == "turn_right"):
                self.turn_right()
            time.sleep(0.01)
            responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())

    def sit(self):
        package = [255,255,1,4,3,2,51,194]
        self.serialDevice.write(package)
    
    def stand(self):
        package = [255,255,1,4,3,2,52,193]
        self.serialDevice.write(package)

    def getup(self):
        package = [255,255,1,4,3,2,104,141]
        self.serialDevice.write(package)

    def left_kick(self):
        package = [255,255,1,4,3,2,106,139]
        self.serialDevice.write(package)
    
    def right_kick(self):
        package = [255,255,1,4,3,2,107,138]
        self.serialDevice.write(package)

    def left_save(self):
        package = [255,255,1,4,3,2,103,142]
        self.serialDevice.write(package)

    def right_save(self):
        package = [255,255,1,4,3,2,102,143]
        self.serialDevice.write(package)

    def forward_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,157,127,127,85]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)

    def left_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,127,157,127,85]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def backward_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,87,127,127,155]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def right_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,127,97,127,145]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def turn_left(self, step_flag = False):
        package = [255,255,1,6,3,5,127,127,167,75]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def turn_right(self, step_flag = False):
        package = [255,255,1,6,3,5,127,127,97,145]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def stop_walk(self):
        package = [255,255,1,4,3,2,110,135]
        self.serialDevice.write(package)