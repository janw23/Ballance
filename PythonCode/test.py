import ImageProcessingModule as IPM
from time import sleep

imageProcessor = IPM.ImageProcessor()

imageProcessor.StartProcessing()

sleep(5)

imageProcessor.StopProcessing()