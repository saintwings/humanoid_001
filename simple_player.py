import multiprocessing
import argparse
import serial
from locomotion_manager import Locomotion
from vision_manager import VisionManager
import time



def check_falling():
    print(locomotion.read_standing_status())

if __name__ == __name__:
    print("Number of cpu : ", multiprocessing.cpu_count())

    parser = argparse.ArgumentParser(description='Enter Comports : con, head, cam' )

    parser.add_argument('--headType',help='motor type on head Mx , Mx ',default='Mx')
    parser.add_argument('-c','--con',help='Controller Comport',default='/dev/ttyACM0')
    parser.add_argument('--head',help='Head Comport',default='/dev/ttyUSB0')
    parser.add_argument('--camera',help='Camera Comport',default='/dev/video0')
    parser.add_argument('--D_version',help='Dynamixel Protocal Version',default='2')

    args = parser.parse_args()


    brain_state = [0,0]

    locomotion = Locomotion(args.con, 115200)
    visionManager = VisionManager(args.head, 115200, args.camera)



    visionManager.enablePanTilt()
    visionManager.setPosition("pan", 0)
    visionManager.setPosition("tilt", 0)
    time.sleep(1)

    #print(visionManager.getPosition("pan"))
    #print(visionManager.getPosition("pan"))

    # visionManager.run_full_scan()

    # print(visionManager.getPosition("pan"))
    # print(visionManager.getPosition("pan"))


    visionManager.open_camera_process()

    time.sleep(2)
    visionManager.run_full_scan()





    # while(True):
    #     check_falling()
    #     visionManager.test_camera(args.camera)
    #     time.sleep(3)




