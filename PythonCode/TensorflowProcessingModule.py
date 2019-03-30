import numpy as np

from tensorflow.contrib.lite.python import interpreter as interpreter_wrapper
import tensorflow as tf

#klasa z funkcjami do przetwarzania danych sieciami neuronowymi z Tensorflow
class TensorflowProcessor:
    
    #sciezka do uzywanego modelu
    model_path = "/home/pi/ballance/ballance_net/ballancenet_conv_2_quant.tflite"
    
    #funkcja generujaca zoptymalizowany model
    def QuantizeModel(model_path, output_file_name):
        print("Quantizing model")
        converter = tf.contrib.lite.TocoConverter.from_saved_model(model_path)
        converter.post_training_quantize = True
        quant_model = converter.convert()
        open(output_file_name + ".tflite", "wb").write(quant_model)
        
    def __init__(self):
        print("Creating TensorflowProcessor object")
        #wczytywanie danego modelu
        self.interpreter = interpreter_wrapper.Interpreter(model_path=TensorflowProcessor.model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        print("TensorflowProcessor object created")
        
    def getPrediction(self, image):
        #przygotowanie obrazu
        image = np.float32(image)
        image /= 255.0
        image = np.expand_dims(image, axis=0)
        image = np.expand_dims(image, axis=3)
        
        #wykonanie interpretacji
        self.interpreter.set_tensor(self.input_details[0]['index'], image)
        self.interpreter.invoke()
        
        return np.squeeze(self.interpreter.get_tensor(self.output_details[0]['index']))