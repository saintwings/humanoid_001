#import multiprocessing
import argparse
import serial
from locomotion_manager import Locomotion
from vision_manager import VisionManager
import time
import threading
import cv2


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
    robot_sub_state = robot_state[1][1]
    if (robot_sub_state == 0):
        for i in range(0,10):
            visionManager.follow_object()
            robot_state[1][1] = 1
    elif (robot_sub_state == 1):
        pass
    

def getup():
    print("getup state")
    time.sleep(2)


if __name__ == "__main__": 
    #print("Number of cpu : ", multiprocessing.cpu_count())

    parser = argparse.ArgumentParser(description='Enter Comports : con, head, cam' )
    parser.add_argument('--headType',help='motor type on head Mx , Mx ',default='Mx')
    parser.add_argument('-c','--con',help='Controller Comport',default='/dev/ttyACM0')
    parser.add_argument('--head',help='Head Comport',default='/dev/ttyUSB0')
    parser.add_argument('--camera',help='Camera Comport',default='/dev/video0')
    parser.add_argument('--D_version',help='Dynamixel Protocal Version',default='2')
    args = parser.parse_args()

    #####-  -#####
    robot_state = [None,[0,0,0,0],[None, None], 0] 
    ## robot_state = [standing, state[main, sub, b_main, b_sub], object_position[x,y], check_sure_object]

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
    locomotion.open_standing_tracking_process()
    robot_report = threading.Thread(target=report_robot_state,args=())
    robot_report.start()


    time.sleep(2)



    robot_state[1][0] = 1



    while(True):
        robot_main_state = robot_state[1][0]
        robot_standing_state = robot_state[0]

        #####- check state -#####
        if (robot_main_state == 1): scan_ball()
        elif (robot_main_state == 2): follow_ball()
        else: just_stand()
        
        #####- falling -#####
        if (robot_standing_state == 1 or robot_standing_state == 2):
            getup()
        




