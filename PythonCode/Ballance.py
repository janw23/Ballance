import ImageProcessingModule as IPM
import ServoControllerModule as SCM
import PIDControllerModule as PIDCM

from time import sleep

imageProcessor = IPM.ImageProcessor()
servoController = SCM.ServoController()
pidController = PIDCM.PIDController()

imageProcessor.StartProcessing()
    
sleep(5)

imageProcessor.StopProcessing()