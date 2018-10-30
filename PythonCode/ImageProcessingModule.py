from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

from imutils.video.pivideostream import PiVideoStream
import imutils
import numpy as np

from multiprocessing import Process
from multiprocessing import Value
 
class ImageProcessor(object):
    
    #stale
    camera_resolution = (112, 112)
    camera_framerate = 40
        
    def __init__(self):
        print("ImageProcessor object created")
        
    def StartProcessing(self):   #uruchamia proces przetwarzajacy obraz
        print("Starting image processing")
        #wartosci-rezultaty przetwarzania obrazu
        self.result_x = Value('d', 0.0)
        self.result_y = Value('d', 0.0)
        
        self.process = Process(target=ImageProcessor.ProcessImage, args=(self,))
        self.process.daemon = True
        self.process.start()
        
        
    def StopProcessing(self):    #wydaje polecenie do zatrzymania przetwarzania obrazu
        print("Stopping image processing")
        self.process.terminate()
        
        
    def ProcessImage(self):    #przetwarza obraz pobierajac klatke z kamery i wykonujac na niej operacje
        self.videoStream = PiVideoStream(resolution=ImageProcessor.camera_resolution, framerate=ImageProcessor.camera_framerate).start()   #uruchamianie watku, ktory w sposob ciagly czyta kolejne klatki z kamery w osobnym watku
        time.sleep(1)
        
        lastTime = time.time()
        a = 0
        while True:
            a = a + 1
            
            if a > 100:
                print(str(a * 1.0 / (time.time() - lastTime)))
                lastTime = time.time()
                a = 0
            
            self.frame = self.videoStream.read() #zapisywanie otrzymanego zdjecia jako tablicy
            
            self.result = ImageProcessor.FindBall(self, self.frame)   #znajdowanie kulki na obrazie i zwracanie rezultatu
            self.result_x = self.result[0]    #ustawianie odpowiedzi w wartosciach dzielonych miedzy procesami
            self.result_y = self.result[1]
            
            cv2.imshow("Frame", self.frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                break
            
        self.videoStream.stop()
            
    def FindBall(self, image):
        return (5.76, 3.865)
 