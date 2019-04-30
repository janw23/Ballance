import time
import zmq
import math
from time import sleep
import MathModule as MM
from multiprocessing import Process, RawValue

import cv2
import numpy as np

#program do komunikacji z symulacja w Unity3D
class SimulationCommunicator:
    
    def __init__(self):
        print("SimulationCommunicator object created")
        
        #wartosci-rezultaty dzialania symulacji
        self.result_x = RawValue('f', 0.0)
        self.result_y = RawValue('f', 0.0)
        
        self.servo_x = RawValue('i', 0)
        self.servo_y = RawValue('i', 0)
        
        #zmienne wartosci
        self.servo_actual_pos = [0, 0]    #aktualna pozycja serwa
        self.servo_target_pos = [0, 0]    #docelowa pozycja serwa
        
        self.refreshDeltaTime = 1 / 60
        
    def getBallPosition(self):    #zwraca pozycje kulki
        return (self.result_x.value, self.result_y.value)

    def StartProcessing(self):   #uruchamia proces przetwarzajacy obraz
        print("Starting simulation communication")
        
        self.process = Process(target=SimulationCommunicator.DoCommunication, args=(self,))
        self.process.daemon = True
        self.process.start()
        
    def StopProcessing(self):
        print("Killing simulation communicator")
        self.servo_x.value = -999
        
    def moveServos(self, tab):
        self.servo_x.value = tab[0]
        self.servo_y.value = tab[1]
                
    def DoCommunication(self):
        timeRefreshed = time.perf_counter()
        
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")
        
        while True:
            if time.perf_counter() - timeRefreshed >= self.refreshDeltaTime:
                timeRefreshed = time.perf_counter()
                
                message = socket.recv_multipart()
                cameraFrame = message[0]
                ballAndCorners = message[1].split()
                #print(str(ballAndCorners))
                
                cameraFrame = np.frombuffer(cameraFrame, np.uint8)
                cameraFrame = np.reshape(cameraFrame, (256, 256, 3))
                cameraFrame = cv2.cvtColor(cameraFrame, cv2.COLOR_RGB2BGR)
                cameraFrame = cv2.flip(cameraFrame, 0)
                #print(str(cameraFrame))
                cv2.imshow("Frame", cameraFrame)
                cv2.waitKey(1)
                
                self.result_x.value = float(ballAndCorners[0]) / 10000.0
                self.result_y.value = float(ballAndCorners[1]) / 10000.0
                    
                servo_x = self.servo_x.value
                socket.send_string(str(servo_x) + " " + str(self.servo_y.value))
                if servo_x == -999:
                    break
                
                sleep(0.01)
