from picamera.array import PiRGBArray
from picamera import PiCamera
import MathModule as MM
import time
import cv2
import copy
from scipy.signal import convolve2d
import matplotlib.pyplot as plt

from imutils.video.pivideostream import PiVideoStream
import imutils
import numpy as np

from multiprocessing import Process
from multiprocessing import Value
 
class ImageProcessor:
    
    #stale
    camera_resolution = (400, 400)
    camera_framerate = 40
    
    corner_detecton_area = (0.09, 0.09, 0.09, 0.09) #prostakat, w ktorym szukana jest krawedz plyty, jest on powielany dla kazdego rogu
    detection_image_resolution = (150, 150)
    
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
            a = a + 1
            
            if a > 100:
                print(str(a * 1.0 / (time.time() - lastTime)))
                lastTime = time.time()
                a = 0
            
            self.frame_original = self.videoStream.read() #zapisywanie otrzymanego zdjecia jako tablicy
            self.frame_original = copy.copy(self.frame_original)
            #self.frame_original = cv2.resize(self.frame_original, (200, 200))
            #self.frame = copy.copy(self.frame_original)
            
            self.corners = ImageProcessor.FindBoardCorners(self)    #znajdowanie pozycji rogow
            #cv2.polylines(self.frame, [self.corners],True,(0,255,255))
            
            ImageProcessor.ChangePerspective(self)
            self.frame_original = self.frame_original[10:141, 10:141]
            self.result = ImageProcessor.FindBall(self)   #znajdowanie kulki na obrazie i zwracanie rezultatu
            
            self.result_x.value = self.result[0] / ImageProcessor.detection_image_resolution[0]   #ustawianie odpowiedzi w wartosciach dzielonych miedzy procesami
            self.result_y.value = self.result[1] / ImageProcessor.detection_image_resolution[1]
            
            cv2.imshow("Frame Casted", self.frame_original)
            #cv2.imshow("Frame", self.frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                break
            
        self.videoStream.stop()
            
    #znajduje pozycje kulki
    def FindBall(self):
        hsv = cv2.cvtColor(self.frame_original, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ImageProcessor.yellow_lower, ImageProcessor.yellow_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        center = (-666.0, -666.0)
        M = cv2.moments(mask)
        if M['m00'] > 0:
            center = (M['m10']/M['m00'], M['m01']/M['m00'])
            cv2.circle(self.frame_original, (int(center[0]), int(center[1])), 1, (0, 0, 255), -1)    #powoduje zle wykrywanie kulki, bo nalozone kolo przechodzi czasami do kolejnej klatki
            	
        return center
    
    #znajduje pozycje krawedzi plyty
    def FindBoardCorners(self):
        corners = [(0, 0), (0, 0), (0, 0), (0, 0)]
        X = (1, 0, -1, 0)
        Y = (0, 1, 0, -1)
        corner_detection_area_pixels = (round(self.corner_detecton_area[0] * self.camera_resolution[0]),
                                       round(self.corner_detecton_area[1] * self.camera_resolution[1]),
                                       round(self.corner_detecton_area[2] * self.camera_resolution[0]),
                                       round(self.corner_detecton_area[3] * self.camera_resolution[1]))
        for i in range(4):
            detectionArea = list(corner_detection_area_pixels)    #domyslnie lewy gorny
            if i == 1 or i == 2:
                detectionArea[0] = self.camera_resolution[0] - detectionArea[0] - detectionArea[2]
            if i == 3 or i == 2:
                detectionArea[1] = self.camera_resolution[1] - detectionArea[1] - detectionArea[3]
                
            rect = (detectionArea[0], detectionArea[1], detectionArea[0] + detectionArea[2], detectionArea[1] + detectionArea[3])
            #cv2.rectangle(self.frame, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 1);
        
            img = self.frame_original[rect[1]:rect[3]+1, rect[0]:rect[2]+1]
            #img = copy.copy(img)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            #hist = cv2.calcHist([hsv], [1, 2], None, [256, 256], [0, 256, 0, 256])
            
            if i == 0: dir = (False, False)
            elif i == 1: dir = (False, True)
            elif i == 2: dir = (True, True)
            elif i == 3: dir = (True, False)
            
            lower = (0, 0, 0)
            upper = (180, 200, 125)
            mask = cv2.inRange(hsv, lower, upper)
            #mask = cv2.dilate(mask, None, iterations=1)
            #mask = cv2.erode(mask, None, iterations=1)
            
            sizeX = np.size(img, 0)
            sizeY = np.size(img, 1)
            
            #neighbour_count_min = 6 * 255    #minimalna liczba sasiadow potrzebna do rozwazenia wierzcholka
            #neighbour_count = np.zeros((sizeX, sizeY))
            #neighbour_count = convolve2d(mask, np.ones((3,3),dtype=int),'same')    #zliczanie liczby sasiadow
            mask = np.int32(mask)
            #img[mask == 255] = [0, 255, 0]
            
            #znajdowanie pozycji rogow
            #mask = np.zeros((sizeX, sizeY))
            #mask[neighbour_count > neighbour_count_min] = 1
            
            row = np.sum(mask, axis=1)
            col = np.sum(mask, axis=0)
            row = np.diff(row)
            col = np.diff(col)
            
            cornerPos = [0.0, 0.0]
            if dir[0]:
                row[0] = row[-1] = 99999999
                cornerPos[1] = np.argmin(row)
            else:
                row[0] = row[-1] = -99999999
                cornerPos[1] = np.argmax(row)
            if dir[1]:
                col[0] = col[-1] = 99999999
                cornerPos[0] = np.argmin(col)
            else:
                col[0] = col[-1] = -99999999
                cornerPos[0] = np.argmax(col)
            
            cornerPos[0] += detectionArea[0]
            cornerPos[1] += detectionArea[1]
            corners[i] = tuple(cornerPos)
            #cv2.imshow("Corner " + str(i), img)
            #cv2.imshow("Corner hist " + str(i), hist)

        return np.array(corners, np.int32)

    #zmienia perspektywe obrazu z kamery tak, aby byla dopasowana do "tablicy"
    def ChangePerspective(self):
        pts = np.array(self.corners, np.float32)
        pts2 = np.float32([[0,0],[self.detection_image_resolution[0],0],[self.detection_image_resolution[0], self.detection_image_resolution[1]], [0,self.detection_image_resolution[1]]])

        M = cv2.getPerspectiveTransform(pts, pts2)
        self.frame_original = cv2.warpPerspective(self.frame_original, M, self.detection_image_resolution)