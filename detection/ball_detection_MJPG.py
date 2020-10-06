import argparse
import time

from PIL import Image
from PIL import ImageDraw

import detect
import tflite_runtime.interpreter as tflite
import platform

import cv2
import numpy as np
import imutils
import math
import v4l2capture
import os
import select

width, height = 1920, 1080
#width, height = 1280, 720
#width, height = 800, 600
WINDOW_NAME = "Preview"
MODEL = 'models/output_tflite_graph_edgetpu.tflite'
LABEL = 'models/label.txt'
THRESHOLD = 0.4
SCAN_BBOX = True

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

def scan(image, interpreter, scan_list, scan_size):
    obj_list = []
    for box in scan_list:
        crop_image = image.crop((box[0], box[1], box[0]+scan_size, box[1]+scan_size))
        scale = detect.set_input(interpreter, (scan_size, scan_size), lambda size: crop_image.resize(size, Image.ANTIALIAS))
        interpreter.invoke()
        objs = detect.get_output(interpreter, THRESHOLD, scale)
        for obj in objs:
            bbox = obj.bbox.translate(box[0], box[1])
            obj_list.append(detect.Object(obj.id, obj.score, bbox))
    return obj_list
def main():
  labels = load_labels(LABEL)
  interpreter = make_interpreter(MODEL)
  interpreter.allocate_tensors()
  TRACKING = False
  full_screen = False
  cap = v4l2capture.Video_device("/dev/video0")
  size_x, size_y = cap.set_format(width, height, fourcc='MJPG')
  cap.create_buffers(1)
  cap.queue_all_buffers()
  cap.start()
  #fourcc = cv2.VideoWriter_fourcc(*'MP4V')
  #out = cv2.VideoWriter('output.mp4', fourcc, 3.0, (width,height))
  while True:
    select.select((cap,), (), ())
    image_data = cap.read_and_queue()
    img = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(image) # image.size = (width, height)
    if image.size != (width, height):
        img = imutils.resize(img, height=height, width=width)
        image = image.resize((width, height))
    ##############################
    ### Step-1 : Overview scan ###
    ##############################
    if not TRACKING:
        scan_list = [(0, 0), (width-height, 0)] # Overview left and right
        scan_size = height
        obj_list = scan(image, interpreter, scan_list, scan_size)
        ## Visualize
        # for obj in obj_list:
        #     # if obj.id != 0: continue # if not ball
        #     bbox = obj.bbox
        #     img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 0, 255), -1)
        #     img = cv2.putText(img, '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), (bbox.xmin + 10, bbox.ymin + 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255), 1)
    ##########################
    ### Step-2 : Fine scan ###
    ##########################
    # Select scan mode
    ids = [obj.id for obj in obj_list]
    FINE_SCAN_MODE = True
    for obj in obj_list:
        if obj.id == 0: # pick the ball (First ball)
            FINE_SCAN_MODE = False
            ball_obj = obj
            break
    ##############################
    ### Step-2.1 : Brute force ###
    ##############################
    if FINE_SCAN_MODE and not TRACKING:
        scan_size = 300
        num_scan_vertical = math.ceil(height/scan_size)
        num_scan_horizontal = math.ceil(width/scan_size)
        step_vertical = int(height/num_scan_vertical)
        step_horizontal = int(width/num_scan_horizontal)
        scan_list = []
        for x_step in range(num_scan_horizontal):
            for y_step in range(num_scan_vertical):
                (x, y) = (x_step*step_vertical, y_step*step_horizontal)
                ## Avoid quantize error in last step
                if x_step == num_scan_horizontal-1: x = width - step_horizontal
                if y_step == num_scan_vertical-1: y = height - step_vertical
                scan_list.append((x, y))
                ## Visualize scan area
                if SCAN_BBOX: img = cv2.rectangle(img, (x, y), (x+scan_size, y+scan_size), (0, 255, 0), 1)
        ## Scan
        obj_list += scan(image, interpreter, scan_list, scan_size)
        ## Visualize & Update tracking variable
        lockon = False
        for obj in obj_list:
            if obj.id != 0: continue # if not ball
            TRACKING = True # Activate tracking
            lockon = True
            ball_obj = obj # Update tracking ball
            bbox = obj.bbox
            img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 0, 255), -1)
            img = cv2.putText(img, '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), (bbox.xmin + 10, bbox.ymin + 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255), 1)
    ###############################
    ### Step-2.2 : Dynamic scan ###
    ###############################
    else:
        # Make dynamic scan area double size as detected ball
        scan_size = max(2 * max(ball_obj.bbox.width, ball_obj.bbox.height), 300) # Use widthest side as scan size (minimum scan_size is 300)
        xmin = int(ball_obj.bbox.xmin - scan_size/4)
        ymin = int(ball_obj.bbox.ymin - scan_size/4)
        xmax = int(xmin + scan_size)
        ymax = int(ymin + scan_size)
        dynamic_bbox = detect.BBox(xmin, ymin, xmax, ymax)
        dynamic_bbox = dynamic_bbox.fit(width, height) # Avoid BBox out of range(width, height)
        obj_list = scan(image, interpreter, [dynamic_bbox], scan_size)
        ## Visualize scan area
        if SCAN_BBOX: img = cv2.rectangle(img, (dynamic_bbox.xmin, dynamic_bbox.ymin), (dynamic_bbox.xmax, dynamic_bbox.ymax), (0, 255, 0), 1)
        ## Visualize & Update tracking variable
        lockon = False
        for obj in obj_list:
            if obj.id != 0: continue # Filter only ball
            TRACKING = True # Activate tracking
            lockon = True
            ball_obj = obj # Update tracking ball
            bbox = obj.bbox
            img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 0, 255), -1)
            img = cv2.putText(img, '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), (bbox.xmin + 10, bbox.ymin + 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255), 1)
        if lockon == False: TRACKING = False # Deactivate TRACKING variable if lock released
    cv2.imshow(WINDOW_NAME, img)
    #out.write(img)
    # cv2.imshow(WINDOW_NAME, cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
    key = cv2.waitKey(1)
    if key == ord('q'):
      break
    elif key == ord('f'):
      full_screen = not full_screen
      if full_screen: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
      else: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
  #out.release()
  cap.release()
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
