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

from PIL import Image
from PIL import ImageDraw

import detect
import tflite_runtime.interpreter as tflite
import platform
import subprocess

import cv2
import numpy as np
import imutils
import v4l2capture
import os
import select



#################################################################

#width, height = 1920, 1080
width, height = 1280, 720
WINDOW_NAME = "Preview"
full_screen = False
MODEL = 'models/output_tflite_graph_edgetpu.tflite'
LABEL = 'models/label.txt'
THRESHOLD = 0.4
COUNT = 1 # Number of times to run inference

def load_labels(path, encoding='utf-8'): #Returns: Dictionary mapping indices to labels.
  with open(path, 'r', encoding=encoding) as f:
    lines = f.readlines()
    if not lines:
      return {}
    if lines[0].split(' ', maxsplit=1)[0].isdigit():
      pairs = [line.split(' ', maxsplit=1) for line in lines]
      return {int(index): label.strip() for index, label in pairs}
    else:
      return {index: line.strip() for index, line in enumerate(lines)}

def make_interpreter(model_file):
  model_file, *device = model_file.split('@')
  return tflite.Interpreter(
      model_path=model_file,
      experimental_delegates=[
          tflite.load_delegate('libedgetpu.so.1', {'device': device[0]} if device else {})])

def draw_objects(draw, objs, labels):
  """Draws the bounding box and label for each object."""
  for obj in objs:
    bbox = obj.bbox
    draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                   outline='red')
    draw.text((bbox.xmin + 10, bbox.ymin + 10), '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), fill='red')


def ball_detection(robot_state):

  labels = load_labels(LABEL)
  interpreter = make_interpreter(MODEL)
  interpreter.allocate_tensors()
  cap = v4l2capture.Video_device("/dev/video0")
  size_x, size_y = cap.set_format(width, height, fourcc='MJPG')

  subprocess.call(["v4l2-ctl", "-c", "focus_auto=0"]) ##trun off auto focus##
  subprocess.call(["v4l2-ctl", "-c", "white_balance_temperature_auto=0"]) ##trun off auto white_balance##


  cap.create_buffers(1)
  cap.queue_all_buffers()
  cap.start()
  while True:
    
    select.select((cap,), (), ())
    image_data = cap.read_and_queue()
    img = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(image) # image.size = (width, height)
    if image.size != (width, height):
        img = imutils.resize(img, height=height, width=width)
        image = image.resize((width, height))
    scale = detect.set_input(interpreter, (width, height), lambda size: image.resize(size, Image.ANTIALIAS))

    inference_time = 0
    for _ in range(COUNT):
      start = time.perf_counter()
      interpreter.invoke()
      inference_time += time.perf_counter() - start
      objs = detect.get_output(interpreter, THRESHOLD, scale)
      # print('%.2f ms' % (inference_time * 1000))
      draw_objects(ImageDraw.Draw(image), objs, labels)

    #print('-------RESULTS--------')
    if not objs:
        robot_state[2][0] = None
        robot_state[2][1] = None
      #print('No objects detected')
    for obj in objs:
        center_x = (obj.bbox.xmax + obj.bbox.xmin)/2
        center_y = (obj.bbox.ymax + obj.bbox.ymin)/2
        robot_state[2][0] = center_x
        robot_state[2][1] = center_y
        # print(labels.get(obj.id, obj.id))
        # print('  id:    ', obj.id)
        # print('  score: ', obj.score)
        # print('  bbox:  ', obj.bbox)
    cv2.imshow(WINDOW_NAME, img)
    cv2.imshow(WINDOW_NAME, cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
    key = cv2.waitKey(1)
    if key == ord('q'):
      break
    elif key == ord('f'):
      full_screen = not full_screen
      if full_screen: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
      else: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
  cv2.destroyAllWindows()


#################################################################


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
        for i in range(0,5):
            visionManager.follow_object()
        visionManager.update_pantilt_position()
        
        #####- check robot rotation -#####
        robot_pan_angle = robot_state[3][0]
        robot_tilt_angle = robot_state[3][1]
        robot_locomotion_lock_state = robot_state[1][2]

        robot_main_state = robot_state[1][0]
        if(robot_locomotion_lock_state == False and robot_main_state == 2):
            if(robot_pan_angle > 20):
                print(robot_state)
                robot_state[1][3] = "turn_left" ## locomotion command ##  
            elif(robot_pan_angle < -20):
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
            enable_timer = threading.Timer(0.5, enable_locomotion_lock_state)
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
            
            
        time.sleep(0.5)
        robot_state[1][3] = "stop"
        time.sleep(2)

    else:
        print("No ball")
        time.sleep(1)
        robot_state[1][0] = 1
        robot_state[1][1] = 0






def enable_locomotion_lock_state():
    robot_state[1][2] = False

def getup():
    print("getup state")
    robot_state[1][3] = "stop"
    visionManager.setPosition("pan", 0)
    visionManager.setPosition("tilt", 0)

    robot_state[1][0] = 0
    robot_state[1][1] = 0
    
    #time.sleep(0.5)
    robot_state[1][3] = "getup"
    time.sleep(0.5)
    robot_state[1][3] = "stop"
    time.sleep(3)

    robot_state[1][0] = 1
    robot_state[1][1] = 0
    


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
    #visionManager.open_object_tracking_process()

    ball_tracking_process = threading.Thread(target=ball_detection,args=(robot_state,))
    ball_tracking_process.start()
    locomotion.open_run_locomotion_process()
    robot_report = threading.Thread(target=report_robot_state,args=())
    robot_report.start()

    # while True:
    #     time.sleep(2)
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
        




