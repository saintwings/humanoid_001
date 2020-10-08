import time

from PIL import Image
from PIL import ImageDraw

import detect
import tflite_runtime.interpreter as tflite
import platform

import cv2
import numpy as np
import imutils
import v4l2capture
import os
import select

class ObjectDetection():
        def __init__(self, camera_comport, screen_size, robot_state):

        self.camera_comport = camera_comport
        self.screen_size = screen_size
        self.object_position_x = None
        self.object_position_y = None
        self.robot_state = robot_state

        #width, height = 1920, 1080
        self.width, height = 1280, 720
        self.WINDOW_NAME = "Preview"
        self.full_screen = False
        self.MODEL = 'models/output_tflite_graph_edgetpu.tflite'
        self.LABEL = 'models/label.txt'
        self.THRESHOLD = 0.4
        self.COUNT = 1 # Number of times to run inference

def load_labels(self, path, encoding='utf-8'): #Returns: Dictionary mapping indices to labels.
  with open(path, 'r', encoding=encoding) as f:
    lines = f.readlines()
    if not lines:
      return {}
    if lines[0].split(' ', maxsplit=1)[0].isdigit():
      pairs = [line.split(' ', maxsplit=1) for line in lines]
      return {int(index): label.strip() for index, label in pairs}
    else:
      return {index: line.strip() for index, line in enumerate(lines)}

def make_interpreter(self, model_file):
  model_file, *device = model_file.split('@')
  return tflite.Interpreter(
      model_path=model_file,
      experimental_delegates=[
          tflite.load_delegate('libedgetpu.so.1', {'device': device[0]} if device else {})])

def draw_objects(self, draw, objs, labels):
  """Draws the bounding box and label for each object."""
  for obj in objs:
    bbox = obj.bbox
    draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                   outline='red')
    draw.text((bbox.xmin + 10, bbox.ymin + 10), '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), fill='red')
    #####
    center_x = (bbox.xmin + bbox.xmax)/2
    center_y = (bbox.ymin + bbox.ymax)/2
    self.robot_state[2][0] = center_x
    self.robot_state[2][1] = center_y

def ball_tracking(self):
  labels = load_labels(self.LABEL)
  interpreter = make_interpreter(self.MODEL)
  interpreter.allocate_tensors()
  cap = v4l2capture.Video_device(self.camera_comport)
  size_x, size_y = cap.set_format(self.width, self.height, fourcc='MJPG')
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
    for _ in range(self.COUNT):
        start = time.perf_counter()
        interpreter.invoke()
        inference_time += time.perf_counter() - start
        objs = detect.get_output(interpreter, THRESHOLD, scale)

        if not objs:
            self.robot_state[2][0] = None
            self.robot_state[2][1] = None
        else:

      # print('%.2f ms' % (inference_time * 1000))
            draw_objects(ImageDraw.Draw(image), objs, labels)

    # print('-------RESULTS--------')
    # if not objs:
    #   print('No objects detected')
    # for obj in objs:
    #   print(labels.get(obj.id, obj.id))
    #   print('  id:    ', obj.id)
    #   print('  score: ', obj.score)
    #   print('  bbox:  ', obj.bbox)
    # cv2.imshow(WINDOW_NAME, img)
        cv2.imshow(WINDOW_NAME, cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1)
        if key == ord('q'):
        break
        elif key == ord('f'):
        full_screen = not full_screen
        if full_screen: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.destroyAllWindows()

