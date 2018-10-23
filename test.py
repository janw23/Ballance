import ServoControllerModule as SCM
from time import sleep
import math

servoController = SCM.ServoController()

angle = 0.0
speed = 5.0
ran = 300.0

while(True):
    angle += speed * 0.01666666
    
    servoController.moveServo(0, math.sin(angle) * ran)
    servoController.moveServo(1, math.cos(angle) * ran)
    
    servoController.update(0.01666666)
    sleep(0.01666666)