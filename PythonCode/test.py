import ImageProcessingModule as IPM
from time import sleep

imageProcessor = IPM.ImageProcessor()

imageProcessor.StartProcessing()

sleep(50)

imageProcessor.StopProcessing()