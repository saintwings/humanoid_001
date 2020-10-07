#import multiprocessing
import argparse
import serial
from locomotion_manager import Locomotion
from vision_manager import VisionManager
import time
import threading
import cv2



def check_falling():
    print(locomotion.read_standing_status())

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
    robot_state = [0,[0,0,0,0],[None, None], 0] 
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


    visionManager.open_camera_process()

    time.sleep(2)
    visionManager.run_full_scan()





    while(True):

        print("a",robot_state)
        time.sleep(0.5)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
    
    #visionManager.close_camera_process()




