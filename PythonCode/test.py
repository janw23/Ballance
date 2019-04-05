import TensorflowProcessingModule as TPM

model_path = "/home/pi/ballance/ballance_net/ballancenet_conv_3"

TPM.TensorflowProcessor.QuantizeModel(model_path, "ballancenet_conv_3_quant")