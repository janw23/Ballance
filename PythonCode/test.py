#import TensorflowProcessingModule as TPM

#TPM.TensorflowProcessor.QuantizeModel("/home/pi/ballance/ballance_net/ballancenet_conv_4", "ballancenet_conv_4_quant")
import ServoControllerModule as SCM
import time
servoController = SCM.ServoController()
servoController.moveServo(0, 1000)

for i in range(300):
    servoController.update(0.01)
    time.sleep(0.01)