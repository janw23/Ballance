import ImageProcessingModule as IPM
from time import sleep

from picamera.array import PiRGBArray
from picamera import PiCamera

imageProcessor = IPM.ImageProcessor()

imageProcessor.StartProcessing()

sleep(50)

imageProcessor.StopProcessing()