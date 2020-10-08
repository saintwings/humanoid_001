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
import v4l2capture
import os
import select

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

