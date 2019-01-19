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
    camera_resolution = (416, 416)
    camera_resolution_cropped = (396, 396)
    camera_framerate = 40
    
    #parametry wykrywania kulki
    yellow_lower = (20, 40, 40)
    yellow_upper = (40, 255, 255)
        
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
            #a = a + 1
            
            if a > 100:
                print(str(a * 1.0 / (time.time() - lastTime)))
                lastTime = time.time()
                a = 0
            
            
            self.frame = self.videoStream.read() #zapisywanie otrzymanego zdjecia jako tablicy
            #przycinanie klatki
            boundLeft = (self.camera_resolution[0] - self.camera_resolution_cropped[0])//2
            boundRight = self.camera_resolution[0] - boundLeft
            boundDown = (self.camera_resolution[1] - self.camera_resolution_cropped[1])//2
            boundUp = self.camera_resolution[1] - boundDown
            self.frame = self.frame[boundLeft:boundRight, boundDown:boundUp]
            
            self.result = ImageProcessor.FindBall(self)   #znajdowanie kulki na obrazie i zwracanie rezultatu
            
            self.result_x.value = self.result[0] / ImageProcessor.camera_resolution_cropped[0]   #ustawianie odpowiedzi w wartosciach dzielonych miedzy procesami
            self.result_y.value = self.result[1] / ImageProcessor.camera_resolution_cropped[1]
            
            cv2.imshow("Frame", self.frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                break
            
        self.videoStream.stop()
            
    def FindBall(self):
        
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ImageProcessor.yellow_lower, ImageProcessor.yellow_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        center = (-666.0, -666.0)
        M = cv2.moments(mask)
        if M['m00'] > 0:
            center = (M['m10']/M['m00'], M['m01']/M['m00'])
            #cv2.circle(self.frame, (int(center[0]), int(center[1])), 1, (0, 0, 255), -1)    #powoduje zle wykrywanie kulki, bo nalozone kolo przechodzi czasami do kolejnej klatki
            	
        return center