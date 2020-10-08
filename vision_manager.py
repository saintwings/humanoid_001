import serial
import time
import numpy as np
import cv2
import subprocess
import threading
import copy
import imutils

from dynamixel_control2 import Dynamixel
from object_detection import ObjectDetection
from pid_control import PID_Control
#from ball_detection_1 import ObjectDetection_2



screen_size = [640, 480]
pan_angle_limit = [-90, 90]
tilt_angle_limit = [-45, 60]

wait_time_motorMove = 0.5
confirm_object_loop_amount = 10
confirm_object_score_qualifier = 7


#####- defind scan paths  -#####
scan_path_001 = [[0, 0], [-90, 0], 3] ##[start[pan,tilt], final[pan,tilt], step_amount]
scan_path_002 = [[-90, 45], [90, 45], 6] ##[start[pan,tilt], final[pan,tilt], step_amount]
scan_path_003 = [[90, 0], [0, 0], 3] ##[start[pan,tilt], final[pan,tilt], step_amount]

scan_paths = []
scan_paths.append(scan_path_001)
scan_paths.append(scan_path_002)
scan_paths.append(scan_path_003)


#####- PID control Parameters -#####
kp_pan = 0.05
ki_pan = 0
kd_pan = 0.0001

kp_tilt = 0.05
ki_tilt = 0
kd_tilt = 0.0001

dt = 0.02


class VisionManager:
    def __init__(self, com, baud, camera_comport, robot_state):

        self.motors = {"pan":41, "tilt":42}

        self.str_comport = com
        self.baudrate = baud
        self.camera_comport = camera_comport
        self.robot_state = robot_state

        self.dt = dt

        self.connect_dynamixel()
        self.pid_pan = PID_Control(kp_pan, ki_pan, kd_pan, self.dt)
        self.pid_tilt = PID_Control(kp_tilt, ki_tilt, kd_tilt, self.dt)

        self.screen_size = screen_size

        self.screen_center_x = self.screen_size[0]/2
        self.screen_center_y = self.screen_size[1]/2

        


        
    def open_object_tracking_process(self):
        self.objectDetection = ObjectDetection(self.camera_comport, self.screen_size, self.robot_state)
        self.object_tracking_process = threading.Thread(target=self.objectDetection.color_tracking,
                    args=())

        # self.objectDetection = ObjectDetection_2(self.camera_comport, self.screen_size, self.robot_state)
        # self.object_tracking_process = threading.Thread(target=self.objectDetection.ball_tracking,
        #             args=())

        self.object_tracking_process.start()

    
    def close_object_tracking_process(self):
        self.object_tracking_process.join()

    def run_full_scan(self):
        self.run_scan_paths(scan_paths)

    def check_object_wait_motor_move_timeout(self):

        while True:
            self.check_object()
            time.sleep(0.02)
        
        return
    
    def check_object(self):
        if self.robot_state[2][0] != None:
            self.found_something = True
            return 1
        else:
            return 0
    
    def confirm_found_object(self):
        score = 0
        for i in range (0,confirm_object_loop_amount):
            score += self.check_object()
        
        if score > confirm_object_score_qualifier:
            return True
        else:
            return False



    def follow_object(self):
            
        object_position_x = self.robot_state[2][0]
        object_position_y = self.robot_state[2][1]

        if object_position_x == None:
            if ( self.confirm_found_object() == False ):
                print("object lost 1")
                time.sleep(0.8)
                if ( self.confirm_found_object() == False ):
                    print("object lost 2")
                    self.robot_state[1][3] = "stop"
                    self.robot_state[1][0] = 1
                    self.robot_state[1][1] = 0
                    
        else:
            motor_position_x = self.getPosition("pan")
            motor_position_y = self.getPosition("tilt")

            motor_position_next_x = motor_position_x + self.pid_pan.update(self.screen_center_x, object_position_x)
            if (motor_position_next_x > pan_angle_limit[1]): motor_position_next_x = pan_angle_limit[1]
            elif (motor_position_next_x < pan_angle_limit[0]): motor_position_next_x = pan_angle_limit[0]
            motor_position_next_y = motor_position_y + (-1)*self.pid_tilt.update(self.screen_center_y, object_position_y)
            if (motor_position_next_y > tilt_angle_limit[1]): motor_position_next_y = tilt_angle_limit[1]
            elif (motor_position_next_y < tilt_angle_limit[0]): motor_position_next_y = tilt_angle_limit[0]

            self.setPosition("pan", motor_position_next_x)
            self.setPosition("tilt", motor_position_next_y)


        time.sleep(self.dt)


    def check_falling_state(self):
        falling_state = self.robot_state[0]
        if(falling_state == 0):
            return False
        else:
            return True


    def run_scan_paths(self, scan_paths):

        #print(scan_paths)

        self.found_something = False

        
        
        for path in scan_paths:
            #print(path)

            ## set motors ready position ##
            self.setPosition("pan", path[0][0])
            self.setPosition("tilt", path[0][1])
            ## wait motors in position ##
            time.sleep(0.2) 
            
            step_size = [(path[1][0] - path[0][0])/path[2], (path[1][1] - path[0][1])/path[2]]
            
            scan_position = copy.deepcopy(path[0])

            for i in range(path[2] + 1):
                
                
                scan_wait_motor_move = threading.Thread(target=self.check_object_wait_motor_move_timeout)

                #print("i = ", scan_position)
                self.setPosition("pan", scan_position[0])
                self.setPosition("tilt", scan_position[1])

                ## set timeout ##
                
                scan_wait_motor_move.start()
                scan_wait_motor_move.join(timeout = wait_time_motorMove)
                if (self.found_something == True):
                    #print("score = ",self.confirm_found_object())
                    if(self.confirm_found_object()):
                        self.robot_state[1][0] = 2
                        self.robot_state[1][1] = 0
                        break
                
                scan_position[0] += step_size[0]
                scan_position[1] += step_size[1] 

                if (self.check_falling_state()):
                    break
            
            if(self.confirm_found_object() or self.check_falling_state()):
                break
        
    def check_object_x_position(self):
        if(self.robot_state[2][0] != None):
            x_ratio = (self.screen_center_x - self.robot_state[2][0])/self.screen_center_x
            y_ratio = (self.screen_center_y - self.robot_state[2][1])/self.screen_center_y
            return [x_ratio, y_ratio]
        else:
            return [None, None]

    def update_pantilt_position(self):
        self.robot_state[3][0] = self.getPosition("pan")
        self.robot_state[3][1] = self.getPosition("tilt")
    
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

    def setLowerCenterPanTilt(self):
        self.setPosition("pan", 0)
        time.sleep(0.01)
        self.setPosition("tilt", 55)
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
        Lower = (29, 86, 6) ##green
        Upper = (64, 255, 255)
        #Lower = (100, 80, 10)
        #Upper = (200, 130, 250)

        self.cap = cv2.VideoCapture(self.camera_comport)
        self.cap.set(3, self.screen_size[0])
        self.cap.set(4, self.screen_size[1])
        subprocess.call(["v4l2-ctl", "-c", "focus_auto=0"]) ##trun off auto focus##
        subprocess.call(["v4l2-ctl", "-c", "white_balance_temperature_auto=0"]) ##trun off auto white_balance##

        while True:
            ret, frame = self.cap.read()
            #print(frame.shape)
            

            blurred = cv2.GaussianBlur(frame, (11,11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            #print(hsv.shape)
            print("color = ")
            print(hsv[240][320][:])

            mask = cv2.inRange(hsv, Lower, Upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cv2.imshow('mask',mask)
            cv2.imshow('frame',frame)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > 20:
                    cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    self.object_position_x = int(x)
                    self.object_position_y = int(y)
                else:
                    self.object_position_x = None
                    self.object_position_y = None

            else:
                self.object_position_x = None
                self.object_position_y = None

            cv2.imshow('frame',frame)

            #### report output to server ####
            self.robot_state[2][0] = self.object_position_x
            self.robot_state[2][1] = self.object_position_y
            #### report output to server ####

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()


   