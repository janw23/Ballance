from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

from imutils.video.pivideostream import PiVideoStream
import imutils
import numpy as np

from multiprocessing import Process
from multiprocessing import Value
 
class ImageProcessor:
    
    #stale
    camera_resolution = (208, 208)
    camera_framerate = 40
    
    #parametry wykrywania kulki
    yellow_lower = (20, 50, 50)
    yellow_upper = (30, 255, 255)
        
    def __init__(self):
        print("ImageProcessor object created")
        #wartosci-rezultaty przetwarzania obrazu
        self.result_x = Value('f', 0.0)
        self.result_y = Value('f', 0.0)
        
        self.key = Value('i', 0)
        
    def getBallPosition(self):    #zwraca pozycje kulki
        return (self.result_x.value, self.result_y.value)
        
    def StartProcessing(self):   #uruchamia proces przetwarzajacy obraz
        print("Starting image processing")
        
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
            
            self.result = ImageProcessor.FindBall(self)   #znajdowanie kulki na obrazie i zwracanie rezultatu
            
            if self.result[0] != -666:
                self.result_x.value = self.result[0] / ImageProcessor.camera_resolution[0]   #ustawianie odpowiedzi w wartosciach dzielonych miedzy procesami
                self.result_y.value = self.result[1] / ImageProcessor.camera_resolution[1]
            
            cv2.imshow("Frame", self.frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                break
            
        self.videoStream.stop()
            
    def FindBall(self):
        
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv, ImageProcessor.yellow_lower, ImageProcessor.yellow_upper)
        self.mask = cv2.erode(self.mask, None, iterations=2)
        self.mask = cv2.dilate(self.mask, None, iterations=2)
	
        self.center = (-666, -666)
        
        M = cv2.moments(self.mask)
        if M['m00'] > 0:
            self.center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
            cv2.circle(self.frame, self.center, 10, (0, 0, 255), -1)
            	
        return self.center