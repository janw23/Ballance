import numpy as np

print("Importing Tensorflow libraries")
from tensorflow.contrib.lite.python import interpreter as interpreter_wrapper
import tensorflow as tf

#klasa z funkcjami do przetwarzania danych sieciami neuronowymi z Tensorflow
class TensorflowProcessor:
    
    #sciezka do uzywanego modelu
    ball_detector_model_path = "/home/pi/ballance/Ballance/Tensorflow/ballancenet_conv_3_quant.tflite"
    corner_detector_model_path = "/home/pi/ballance/Ballance/Tensorflow/ballancenet_boardcorner_conv_2_quant.tflite"
    
    #funkcja generujaca zoptymalizowany model
    def QuantizeModel(model_path, output_file_name):
        print("Quantizing model")
        converter = tf.contrib.lite.TocoConverter.from_saved_model(model_path)
        converter.post_training_quantize = True
        quant_model = converter.convert()
        open(output_file_name + ".tflite", "wb").write(quant_model)
        
    def __init__(self):
        print("Creating TensorflowProcessor object")
        #wczytywanie modelu do wykrywania kulki
        print("Loading ball detection tflite model")
        self.ball_detector_interpreter = interpreter_wrapper.Interpreter(model_path=TensorflowProcessor.ball_detector_model_path)
        self.ball_detector_interpreter.allocate_tensors()
        self.ball_detector_input_details = self.ball_detector_interpreter.get_input_details()
        self.ball_detector_output_details = self.ball_detector_interpreter.get_output_details()
        
        #wczytywanie modelu do wykrywania krawedzi plyty
        print("Loading corner detection tflite model")
        self.corner_detector_interpreter = interpreter_wrapper.Interpreter(model_path=TensorflowProcessor.corner_detector_model_path)
        self.corner_detector_interpreter.allocate_tensors()
        self.corner_detector_input_details = self.corner_detector_interpreter.get_input_details()
        self.corner_detector_output_details = self.corner_detector_interpreter.get_output_details()
        
        print("TensorflowProcessor object created")
        
    def getBallPosition(self, image):
        #przygotowanie obrazu
        image = np.float32(image)
        image /= 255.0
        image = np.expand_dims(image, axis=0)
        image = np.expand_dims(image, axis=3)
        
        #wykonanie interpretacji
        self.ball_detector_interpreter.set_tensor(self.ball_detector_input_details[0]['index'], image)
        self.ball_detector_interpreter.invoke()
        
        return np.squeeze(self.ball_detector_interpreter.get_tensor(self.ball_detector_output_details[0]['index']))
    
    def getCornerPosition(self, image):
        #przygotowanie obrazu
        image = np.float32(image)
        image /= 255.0
        image = np.expand_dims(image, axis=0)
        image = np.expand_dims(image, axis=3)
        
        #wykonanie interpretacji
        self.corner_detector_interpreter.set_tensor(self.corner_detector_input_details[0]['index'], image)
        self.corner_detector_interpreter.invoke()
        
        return np.squeeze(self.corner_detector_interpreter.get_tensor(self.corner_detector_output_details[0]['index']))