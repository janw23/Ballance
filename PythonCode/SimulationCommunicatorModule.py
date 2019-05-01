import time
import zmq
import math
from time import sleep
import MathModule as MM
from multiprocessing import Process, RawValue, RawArray

import cv2
import numpy as np

#program do komunikacji z symulacja w Unity3D
class SimulationCommunicator:
    
    def __init__(self):
        print("SimulationCommunicator object created")
        
        #wartosci-rezultaty dzialania symulacji
        self.ball_x = RawValue('f', 0.0)
        self.ball_y = RawValue('f', 0.0)
        
        self.servo_x = RawValue('i', 0)
        self.servo_y = RawValue('i', 0)
        
        self.corner_tl_x = RawValue('f', 0.0)
        self.corner_tl_y = RawValue('f', 0.0)
        self.corner_tr_x = RawValue('f', 0.0)
        self.corner_tr_y = RawValue('f', 0.0)
        self.corner_br_x = RawValue('f', 0.0)
        self.corner_br_y = RawValue('f', 0.0)
        self.corner_bl_x = RawValue('f', 0.0)
        self.corner_bl_y = RawValue('f', 0.0)
        
        self.cameraFrame = RawArray('i', 3 * 256**2)
        
        #zmienne wartosci
        self.servo_actual_pos = [0, 0]    #aktualna pozycja serwa
        self.servo_target_pos = [0, 0]    #docelowa pozycja serwa
        
        self.refreshDeltaTime = 1 / 60
        
    def getBallPosition(self):    #zwraca pozycje kulki
        return (self.ball_x.value, self.ball_y.value)
    
    def FindBoardCorners(self):    #zwraca pozycje rogow plyty
        return ((self.corner_tl_x.value, self.corner_tl_y.value),
                (self.corner_tr_x.value, self.corner_tr_y.value),
                (self.corner_br_x.value, self.corner_br_y.value),
                (self.corner_bl_x.value, self.corner_bl_y.value))
    
    def read(self):    #zwraca klatke z symulowanej kamery
        frame = np.frombuffer(self.cameraFrame, dtype=np.int32)
        frame = np.reshape(frame, (256, 256, 3)).astype('uint8')
        return frame
        
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
        
        self.cameraFrame_np = np.frombuffer(self.cameraFrame, dtype=np.int32).reshape(3 * 256**2)
        
        while True:
            if time.perf_counter() - timeRefreshed >= self.refreshDeltaTime:
                timeRefreshed = time.perf_counter()
                
                #odbieranie wiadomosci
                message = socket.recv_multipart()
                cameraFrame = message[0]
                ballAndCorners = message[1].split()
                
                #czytanie odebranej klatki z symulowanej kamery
                cameraFrame = np.frombuffer(cameraFrame, np.uint8)
                if len(cameraFrame) > 0:
                    cameraFrame = np.reshape(cameraFrame, (256, 256, 3))
                    cameraFrame = cv2.cvtColor(cameraFrame, cv2.COLOR_RGB2BGR)
                    cameraFrame = cv2.flip(cameraFrame, 0)
                    #print(str(cameraFrame))
                    #cv2.imshow("Frame", cameraFrame)
                    #cv2.waitKey(1)
                
                    #zapisywanie klatki do dzielonej pamieci
                    cameraFrame = np.int32(cameraFrame)
                    np.copyto(self.cameraFrame_np, cameraFrame.ravel())
                
                #zapisywanie pozycji kulki i rogow w pamieci dzielonej
                self.ball_x.value = float(ballAndCorners[0])
                self.ball_y.value = float(ballAndCorners[1])
                self.corner_tl_x.value = float(ballAndCorners[2]) * 256
                self.corner_tl_y.value = float(ballAndCorners[3]) * 256
                self.corner_tr_x.value = float(ballAndCorners[4]) * 256
                self.corner_tr_y.value = float(ballAndCorners[5]) * 256
                self.corner_br_x.value = float(ballAndCorners[6]) * 256
                self.corner_br_y.value = float(ballAndCorners[7]) * 256
                self.corner_bl_x.value = float(ballAndCorners[8]) * 256
                self.corner_bl_y.value = float(ballAndCorners[9]) * 256
                    
                servo_x = self.servo_x.value
                socket.send_string(str(servo_x) + " " + str(self.servo_y.value))
                if servo_x == -999:
                    break
                
                sleep(0.01)
