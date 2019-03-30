import TensorflowProcessingModule as TPM

import MathModule as MM
import math, time, copy
import cv2

from multiprocessing import Process, Value
from imutils.video.pivideostream import PiVideoStream
import numpy as np
 
#program sluzacy do analizy obrazu z kamery, wykrywania kulki
class ImageProcessor:
    
    #parametry kamery
    camera_resolution = (300, 300)
    camera_framerate = 40
    
    corner_detecton_area = (0.08, 0.08, 0.14, 0.14) #prostakat, w ktorym szukana jest krawedz plyty, jest on powielany dla kazdego rogu obrazu
    detection_image_resolution = (200, 200)
    detection_image_resolution_cropped = (-1, -1)
        
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
        self.key.value = -666
        self.process.terminate()
        
    def ProcessImage(self):    #przetwarza obraz pobierajac klatke z kamery i wykonujac na niej operacje analizy
        
        #parametry trackera kulki
        self.ballTracker_pos = [ImageProcessor.detection_image_resolution[0]//2, ImageProcessor.detection_image_resolution[1]//2]
        self.ballTracker_size = (50, 50)
        self.ballTracker_result = [0, 0]
        
        self.tensorflowProcessor = TPM.TensorflowProcessor()
        videoStream = PiVideoStream(resolution=ImageProcessor.camera_resolution, framerate=ImageProcessor.camera_framerate).start()   #uruchamianie watku, ktory czyta kolejne klatki z kamery
        
        time.sleep(1)
        self.frame_original = videoStream.read()
        
        lastTime = time.time()
        a = 190
        lastID = 0
        
        while True:
            
            #prosty licznik przetworzonych klatek w ciagu sekundy
            a = a + 1
            if a > 200:
                if ImageProcessor.detection_image_resolution_cropped[0] == -1:
                    ImageProcessor.detection_image_resolution_cropped = (np.size(self.frame_original, 0), np.size(self.frame_original, 1))
                print(str(a * 1.0 / (time.time() - lastTime)))
                lastTime = time.time()
                a = 0
            
            #synchronizacja pobierania nowej klatki z czestotliwascia kamery
            while True:
                frameGrabbed = videoStream.read()
                ID = id(frameGrabbed)
                if ID != lastID:
                    self.frame_original = frameGrabbed
                    lastID = ID
                    break
                else:
                    time.sleep(0.01)
            
            self.corners = ImageProcessor.FindBoardCorners(self)    #znajdowanie pozycji rogow plyty
            ImageProcessor.ChangePerspective(self)    #zmiana perspektywy znalezionej tablicy, aby wygladala jak kwadrat
            self.frame_original = self.frame_original[5:196, 5:196] #przycinanie zdjecia
            ImageProcessor.UpdateBallTracker(self)    #aktualizacja trackera kulki
            
            #ustawianie znalezionej pozycji kulki w zmiennych dzielonych miedzy procesami
            self.result_x.value = self.ballTracker_result[0] / ImageProcessor.detection_image_resolution_cropped[0]
            self.result_y.value = self.ballTracker_result[1] / ImageProcessor.detection_image_resolution_cropped[1]
            
            cv2.imshow("Frame Casted", self.frame_original)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or self.key.value == -666:
                break
            
        videoStream.stop()
            
    #aktualizuje tracker kulki
    def UpdateBallTracker(self):
        self.ballTracker_pos[0] = MM.clamp(self.ballTracker_pos[0], 0, ImageProcessor.detection_image_resolution_cropped[0] - self.ballTracker_size[0])
        self.ballTracker_pos[1] = MM.clamp(self.ballTracker_pos[1], 0, ImageProcessor.detection_image_resolution_cropped[1] - self.ballTracker_size[1])
        
        self.ballTracker_pos[0] = int(self.ballTracker_pos[0])
        self.ballTracker_pos[1] = int(self.ballTracker_pos[1])
        
        #przygotowanie klatki z kamery do analizy
        tracker_frame = self.frame_original[self.ballTracker_pos[1]:self.ballTracker_pos[1]+self.ballTracker_size[1],
                                            self.ballTracker_pos[0]:self.ballTracker_pos[0]+self.ballTracker_size[0]]
        tracker_frame = cv2.cvtColor(tracker_frame, cv2.COLOR_BGR2GRAY)
        
        #analiza klatki z uzyciem sieci neuronowych
        result = self.tensorflowProcessor.getPrediction(tracker_frame)
        result = (int(round(result[0] * self.ballTracker_size[0])), int(round(result[1] * self.ballTracker_size[1])))
        
        self.ballTracker_result[0] = self.ballTracker_pos[0] + result[0]
        self.ballTracker_result[1] = self.ballTracker_pos[1] + result[1]
        
        #zaznaczanie wizualne pozycji kulki
        cv2.circle(self.frame_original, tuple(self.ballTracker_result), 2, (0, 0, 255), -1)
        
        #aktualizacja pozycji trackera
        self.ballTracker_pos[0] = MM.lerp(self.ballTracker_pos[0], self.ballTracker_result[0] - self.ballTracker_size[0] // 2, 0.4)
        self.ballTracker_pos[1] = MM.lerp(self.ballTracker_pos[1], self.ballTracker_result[1] - self.ballTracker_size[1] // 2, 0.4)
    
    #znajduje pozycje krawedzi plyty
    def FindBoardCorners(self):
        corners = [(0, 0), (0, 0), (0, 0), (0, 0)]
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
            
            lower = (0, 0, 200)
            upper = (180, 30, 255)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.dilate(mask, None, iterations=1)
            #mask = cv2.erode(mask, None, iterations=1)
            
            center = (0, 0)
            M = cv2.moments(mask)
            if M['m00'] > 0:
                center = (M['m10']/M['m00'], M['m01']/M['m00'])
            
            corners[i] = (center[0] + detectionArea[0], center[1] + detectionArea[1])
            #cv2.imshow("Corner " + str(i), img)
            #cv2.imshow("Corner hist " + str(i), hist)

        return np.array(corners, np.int32)

    #zmienia perspektywe obrazu z kamery tak, aby niewidoczne bylo przechylenie plyty
    def ChangePerspective(self):
        pts = np.array(self.corners, np.float32)
        pts2 = np.float32([[0,0],[self.detection_image_resolution[0],0],[self.detection_image_resolution[0], self.detection_image_resolution[1]], [0,self.detection_image_resolution[1]]])

        M = cv2.getPerspectiveTransform(pts, pts2)
        self.frame_original = cv2.warpPerspective(self.frame_original, M, self.detection_image_resolution)