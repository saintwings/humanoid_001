#import multiprocessing
import sys
import signal
import argparse
import serial
from locomotion_manager import Locomotion
from vision_manager import VisionManager
import time
import threading
import cv2


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    robot_state[1][3] = "stop"
    locomotion.set_locomotion(robot_state[1][3])
    sys.exit(0)

def report_robot_state():
    while True:
        print(robot_state)
        time.sleep(0.2)

def just_stand():
    pass

def scan_ball():
    robot_sub_state = robot_state[1][1]
    if (robot_sub_state == 0):
        visionManager.run_full_scan()
    elif (robot_sub_state == 1):
        time.sleep(2)
        robot_state[1][1] = 0

def follow_ball():
    robot_main_state = robot_state[1][0]
    robot_sub_state = robot_state[1][1]
    if (robot_sub_state == 0):
        for i in range(0,5):
            visionManager.follow_object()
        visionManager.update_pantilt_position()
        robot_state[1][1] = 1
    elif (robot_sub_state == 1):
        for i in range(0,3):
            visionManager.follow_object()
        visionManager.update_pantilt_position()
        
        #####- check robot rotation -#####
        robot_pan_angle = robot_state[3][0]
        robot_tilt_angle = robot_state[3][1]
        robot_locomotion_lock_state = robot_state[1][2]

        robot_main_state = robot_state[1][0]
        if(robot_locomotion_lock_state == False and robot_main_state == 2):
            if(robot_pan_angle > 15):
                print(robot_state)
                robot_state[1][3] = "turn_left" ## locomotion command ##  
            elif(robot_pan_angle < -15):
                robot_state[1][3] = "turn_right" ## locomotion command ##
            else:
                if(robot_tilt_angle < 50):
                    robot_state[1][3] = "forward" ## locomotion command ##
                else:
                    robot_state[1][3] = "stop" ## locomotion command ##
                    robot_state[1][0] = 3 ## goto main_state 3

            #locomotion.set_locomotion(robot_state[1][3])
            #####- Lock Locomotion -#####
            robot_state[1][2] = True
            enable_timer = threading.Timer(1, enable_locomotion_lock_state)
            enable_timer.start()

def prepare_kick():
    visionManager.setLowerCenterPanTilt()
    time.sleep(0.5)
    if robot_state[2][0] != None:
        time.sleep(1)
        x_ratio, y_ratio = visionManager.check_object_x_position()
        print(x_ratio, y_ratio)

        if(x_ratio != None):
            if(y_ratio < 0.2):
                if(x_ratio > 0 and x_ratio < 0.8):
                    print("Left Kick")
                    robot_state[1][3] = "left_kick"
                elif(x_ratio < 0 and x_ratio > -0.8):
                    print("Right Kick")
                    robot_state[1][3] = "right_kick"
                else:
                    robot_state[1][0] = 2
                    robot_state[1][1] = 0
            else:
                robot_state[1][0] = 2
                robot_state[1][1] = 0
            
            

        
        time.sleep(2)
        robot_state[1][3] = "stop"
    else:
        print("No ball")
        time.sleep(1)
        robot_state[1][0] = 1
        robot_state[1][1] = 0






def enable_locomotion_lock_state():
    robot_state[1][2] = False

def getup():
    print("getup state")
    time.sleep(2)
    


if __name__ == "__main__": 
    #print("Number of cpu : ", multiprocessing.cpu_count())

    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description='Enter Comports : con, head, cam' )
    parser.add_argument('--headType',help='motor type on head Mx , Mx ',default='Mx')
    parser.add_argument('-c','--con',help='Controller Comport',default='/dev/ttyACM0')
    parser.add_argument('--head',help='Head Comport',default='/dev/ttyUSB0')
    parser.add_argument('--camera',help='Camera Comport',default='/dev/video0')
    parser.add_argument('--D_version',help='Dynamixel Protocal Version',default='2')
    args = parser.parse_args()

    #####-  -#####
    robot_state = [None,[0,0, False, None,0,0],[None, None, False], [None, None]] 
    ## robot_state = [standing, state[main, sub, lock_locomotion, locomotion_command, b_main, b_sub], object_position[x,y,check_sure_object], pantilt_position[pan,tilt]]

    #####-  -#####
    locomotion = Locomotion(args.con, 115200, robot_state)
    visionManager = VisionManager(args.head, 115200, args.camera, robot_state)

    #####-  -#####
    visionManager.enablePanTilt()
    visionManager.setPosition("pan", 0)
    visionManager.setPosition("tilt", 0)
    time.sleep(1)

    #####-  -#####
    visionManager.open_object_tracking_process()
    locomotion.open_run_locomotion_process()
    robot_report = threading.Thread(target=report_robot_state,args=())
    robot_report.start()

    #locomotion.set_locomotion(robot_state[1][3])
    time.sleep(2)

    


    robot_state[1][3] = "stop"
    robot_state[1][0] = 1



    while(True):
        robot_main_state = robot_state[1][0]
        robot_standing_state = robot_state[0]

        #####- check state -#####
        if (robot_main_state == 1): scan_ball()
        elif (robot_main_state == 2): follow_ball()
        elif (robot_main_state == 3): prepare_kick()
        else: just_stand()
        
        #####- falling -#####
        if (robot_standing_state == 1 or robot_standing_state == 2):
            getup()
        




