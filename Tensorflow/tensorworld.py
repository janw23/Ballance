from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import cv2
from imutils.video.pivideostream import PiVideoStream
from time import sleep
import math

from tensorflow.contrib.lite.python import interpreter as interpreter_wrapper

model_file = "/home/pi/ballance/Ballance/Tensorflow/ballancenet.tflite"
interpreter = interpreter_wrapper.Interpreter(model_path=model_file)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print(input_details)

print("Starting processing")
videoStream = PiVideoStream(resolution=(400, 400), framerate=40).start()
sleep(1)

while True:
    frame = videoStream.read()
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
    frame_cut = frame[170:231, 170:231]
    frame_cut = cv2.resize(frame_cut, (30, 30))
    
    gray = cv2.cvtColor(frame_cut, cv2.COLOR_BGR2GRAY)
    gray = gray / 255.0
    gray = np.expand_dims(gray, axis=0)
    
    interpreter.set_tensor(input_details[0]['index'], np.float32(gray))
    interpreter.invoke()
    
    output_data = interpreter.get_tensor(output_details[0]['index'])
    results = np.squeeze(output_data)
    
    pos = (int(round(results[1] * 30)), int(round(results[0] * 30)))
    cv2.circle(frame_cut, pos, 1, (0, 0, 255), -1)
    
    frame_cut = cv2.resize(frame_cut, (200, 200))
    cv2.imshow("Frame cut", frame_cut)
    