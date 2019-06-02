#import TensorflowProcessingModule as TPM

#TPM.TensorflowProcessor.QuantizeModel("/home/pi/ballance/ballance_net/ballancenet_conv_4", "ballancenet_conv_4_quant")
import MathModule as MM

data = [1, 5, 10, -20]
print(str(MM.Median(data)))