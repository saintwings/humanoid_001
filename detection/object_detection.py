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

EDGETPU_SHARED_LIB = {
  'Linux': 'libedgetpu.so.1',
  'Darwin': 'libedgetpu.1.dylib',
  'Windows': 'edgetpu.dll'
}[platform.system()]


def load_labels(path, encoding='utf-8'):
  """Loads labels from file (with or without index numbers).

  Args:
    path: path to label file.
    encoding: label file encoding.
  Returns:
    Dictionary mapping indices to labels.
  """
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
          tflite.load_delegate(EDGETPU_SHARED_LIB,
                               {'device': device[0]} if device else {})
      ])


def draw_objects(draw, objs, labels):
  """Draws the bounding box and label for each object."""
  for obj in objs:
    bbox = obj.bbox
    draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                   outline='red')
    draw.text((bbox.xmin + 10, bbox.ymin + 10),
              '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score),
              fill='red')


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('-m', '--model', default='models/output_tflite_graph_edgetpu.tflite')
  parser.add_argument('-l', '--labels', default='models/label.txt')
  parser.add_argument('-t', '--threshold', type=float, default=0.4)
  parser.add_argument('-c', '--count', type=int, default=5, help='Number of times to run inference')
  args = parser.parse_args()

  labels = load_labels(args.labels) if args.labels else {}
  interpreter = make_interpreter(args.model)
  interpreter.allocate_tensors()

  cap = cv2.VideoCapture(0)

  while True:
    ret, img = cap.read()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(image)
    scale = detect.set_input(interpreter, image.size, lambda size: image.resize(size, Image.ANTIALIAS))
    # print('----INFERENCE TIME----')
    # print('Note: The first inference is slow because it includes',
    #       'loading the model into Edge TPU memory.')
    for _ in range(args.count):
      start = time.perf_counter()
      interpreter.invoke()
      inference_time = time.perf_counter() - start
      objs = detect.get_output(interpreter, args.threshold, scale)
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

    cv2.imshow("Preview", cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))

    cv2.waitKey(10)

  image = Image.open(args.input)
  scale = detect.set_input(interpreter, image.size,
                           lambda size: image.resize(size, Image.ANTIALIAS))

  print('----INFERENCE TIME----')
  print('Note: The first inference is slow because it includes',
        'loading the model into Edge TPU memory.')
  for _ in range(args.count):
    start = time.perf_counter()
    interpreter.invoke()
    inference_time = time.perf_counter() - start
    objs = detect.get_output(interpreter, args.threshold, scale)
    print('%.2f ms' % (inference_time * 1000))

  print('-------RESULTS--------')
  if not objs:
    print('No objects detected')

  for obj in objs:
    print(labels.get(obj.id, obj.id))
    print('  id:    ', obj.id)
    print('  score: ', obj.score)
    print('  bbox:  ', obj.bbox)

  if args.output:
    image = image.convert('RGB')
    draw_objects(ImageDraw.Draw(image), objs, labels)
    image.save(args.output)
    image.show()


if __name__ == '__main__':
  main()
