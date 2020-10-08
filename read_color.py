import cv2
from vision_manager import VisionManager
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Enter Comports : con, head, cam' )
    parser.add_argument('--headType',help='motor type on head Mx , Mx ',default='Mx')
    parser.add_argument('-c','--con',help='Controller Comport',default='/dev/ttyACM0')
    parser.add_argument('--head',help='Head Comport',default='/dev/ttyUSB0')
    parser.add_argument('--camera',help='Camera Comport',default='/dev/video0')
    parser.add_argument('--D_version',help='Dynamixel Protocal Version',default='2')
    args = parser.parse_args()

    robot_state = [None,[0,0, False, None,0,0],[None, None, False], [None, None]] 


    visionManager = VisionManager(args.head, 115200, args.camera, robot_state)
    visionManager.test_camera(args.camera)
