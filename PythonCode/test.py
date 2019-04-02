import TensorflowProcessingModule as TPM

model_path = "/home/pi/ballance/ballance_net/ballancenet_boardcorner_conv2"

TPM.TensorflowProcessor.QuantizeModel(model_path, "ballancenet_boardcorner_conv_2_quant")