#import TensorflowProcessingModule as TPM

#TPM.TensorflowProcessor.QuantizeModel("/home/pi/ballance/ballance_net/ballancenet_conv_4", "ballancenet_conv_4_quant")
import MathModule as MM
import ServoControllerModule as SCM
from time import sleep

servoController = SCM.ServoController()

servoController.moveServo(0, 0)
servoController.moveServo(1, 0)
servoController.update(100000)

for i in range(20):
    servoController.moveServo(1, 400)
    servoController.update(100000)
    sleep(3)
    servoController.moveServo(1, 0)
    servoController.update(100000)
    sleep(3)
    servoController.moveServo(1, -400)
    servoController.update(100000)
    sleep(3)
    servoController.moveServo(1, 0)
    servoController.update(100000)
    sleep(3)