import serial
import time
import numpy as np
import cv2
import subprocess
import multiprocessing 
from dynamixel_control2 import Dynamixel
from object_detection import ObjectDetection

scan_path_001 = [[-90, 0], [90, 0]] #[pan, tilt]
scan_path_002 = [[90, 45], [-90, 45]] #[pan, tilt]
scan_step_amount = 6

scan_paths = []
scan_paths.append(scan_path_001)
scan_paths.append(scan_path_002)

class VisionManager:
    def __init__(self, com, baud, camera_comport):

        self.motors = {"pan":41, "tilt":42}


        self.str_comport = com
        self.baudrate = baud
        self.camera_comport = camera_comport

        self.connect_dynamixel()


        

    def open_camera_process(self):
        self.objectDetection = ObjectDetection(self.camera_comport)
        self.camera_process = multiprocessing.Process(target=self.objectDetection.color_tracking,
                    args=())
        
        self.camera_process.start()
    
    def close_camera_process(self):
        self.camera_process.join()

    def run_full_scan(self):
        self.run_scan_paths(scan_paths, scan_step_amount)

    def run_scan_paths(self, scan_paths, scan_step_amount):

    
        for path in scan_paths:
            ## set motors ready position ##
            self.setPosition("pan", path[0][0])
            self.setPosition("tilt", path[0][1])
            ## wait motors in position ##
            time.sleep(0.2) 

            step_size = [(path[1][0] - path[0][0])/scan_step_amount, (path[1][1] - path[0][1])/scan_step_amount]
            
            scan_position = path[0]
            for i in range(scan_step_amount + 1):
                print("i = ", scan_position)
                self.setPosition("pan", scan_position[0])
                self.setPosition("tilt", scan_position[1])
                time.sleep(0.25)
                
                ## capture ##

 
                scan_position[0] += step_size[0]
                scan_position[1] += step_size[1] 
        
        self.close_camera_process()
        
    
    
                
            
    
    def connect_dynamixel(self):
        self.dynamixel = Dynamixel(self.str_comport, self.baudrate)




    def enablePanTilt(self):
        self.dynamixel.setEnableMotorTurque(self.motors["pan"])
        time.sleep(0.01)
        self.dynamixel.setEnableMotorTurque(self.motors["tilt"])
        time.sleep(0.01)
    
    def setCenterPanTilt(self):
        self.setPosition("pan", 0)
        time.sleep(0.01)
        self.setPosition("tilt", 0)
        time.sleep(0.01)

    def setPosition(self, motor, value):
        if(value >= -180 and value < 180):
            motor_value = int(self.convertAngleDegToMotorValue(value))
            self.dynamixel.setDeviceMoving(self.motors[motor],"MX",motor_value,1024,1024)
        else:
            print("Value out of bound")
        time.sleep(0.01)

    def getPosition(self, motor):
        
        motor_value = self.dynamixel.getMotorPosition(self.motors[motor])
        position = self.convertMotorValueToAngleDeg(motor_value)

        return position
    
    def convertMotorValueToAngleDeg(self, motor_value):
        angle_deg = (motor_value * 360 / 4096) - 180
        return angle_deg

    def convertAngleDegToMotorValue(self, angle_deg):
        motor_value = (angle_deg + 180) * 4096 / 360
        return motor_value


    def test_camera(self, camera_comport):
        cap = cv2.VideoCapture(camera_comport)
        cap.set(3, 640)
        cap.set(4, 480)
        subprocess.call(["v4l2-ctl", "-c", "focus_auto=0"]) ##trun off auto focus##

        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()


            # Display the resulting frame
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

