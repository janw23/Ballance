import TensorflowProcessingModule as TPM

import MathModule as MM
import math, time, copy
import cv2

from multiprocessing import Process, Value
from imutils.video.pivideostream import PiVideoStream
import numpy as np

import profile
 
#program sluzacy do analizy obrazu z kamery, wykrywania kulki
class ImageProcessor:
    
    #parametry kamery
    camera_resolution = (256, 256)
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
        #ImageProcessor.ProcessImage(self)
        
    def StopProcessing(self):    #wydaje polecenie do zatrzymania przetwarzania obrazu
        print("Stopping image processing")
        self.key.value = -666
        self.process.terminate()
        
    def ProcessImage(self):    #przetwarza obraz pobierajac klatke z kamery i wykonujac na niej operacje analizy
        
        #parametry trackera kulki
        self.ballTracker_pos = [ImageProcessor.detection_image_resolution[0]//2, ImageProcessor.detection_image_resolution[1]//2]
        self.ballTracker_size = 40
        self.ballTracker_result = [0, 0]
        
        self.tensorflowProcessor = TPM.TensorflowProcessor()
        videoStream = PiVideoStream(resolution=ImageProcessor.camera_resolution, framerate=ImageProcessor.camera_framerate).start()   #uruchamianie watku, ktory czyta kolejne klatki z kamery
        
        time.sleep(1)
        self.frame_original = videoStream.read()
        
        lastTime = time.time()
        a = 190
        lastID = 0
        
        saveCounter = 0
        saveCount = 0
        
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
            
            #klatka przeznaczona do debugowania
            #self.frame_debug = copy.copy(self.frame_original)
            
            self.corners = ImageProcessor.FindBoardCorners(self)    #znajdowanie pozycji rogow plyty
            ImageProcessor.ChangePerspective(self)    #zmiana perspektywy znalezionej tablicy, aby wygladala jak kwadrat
            self.frame_original = self.frame_original[2:199, 2:199] #przycinanie zdjecia
            ImageProcessor.UpdateBallTracker(self)    #aktualizacja trackera kulki
            
            #ustawianie znalezionej pozycji kulki w zmiennych dzielonych miedzy procesami
            self.result_x.value = self.ballTracker_result[0] / ImageProcessor.detection_image_resolution_cropped[0]
            self.result_y.value = self.ballTracker_result[1] / ImageProcessor.detection_image_resolution_cropped[1]
            
            #cv2.imshow("Frame debug", self.frame_debug)
            if saveCounter < saveCount:
                cv2.imwrite("Frame" + str(saveCounter) + ".png", self.frame_original)
                saveCounter += 1
                
            #cv2.imshow("Frame Casted", self.frame_original)
            #key = cv2.waitKey(1) & 0xFF
            #if key == ord("q") or self.key.value == -666:
            #    break
            
        videoStream.stop()
            
    #aktualizuje tracker kulki
    def UpdateBallTracker(self):
        self.ballTracker_pos[0] = MM.clamp(self.ballTracker_pos[0], 0, ImageProcessor.detection_image_resolution_cropped[0] - self.ballTracker_size)
        self.ballTracker_pos[1] = MM.clamp(self.ballTracker_pos[1], 0, ImageProcessor.detection_image_resolution_cropped[1] - self.ballTracker_size)
        
        self.ballTracker_pos[0] = int(self.ballTracker_pos[0])
        self.ballTracker_pos[1] = int(self.ballTracker_pos[1])
        
        #przygotowanie klatki z kamery do analizy
        tracker_frame = self.frame_original[self.ballTracker_pos[1]:self.ballTracker_pos[1]+self.ballTracker_size,
                                            self.ballTracker_pos[0]:self.ballTracker_pos[0]+self.ballTracker_size]
        tracker_frame = cv2.cvtColor(tracker_frame, cv2.COLOR_BGR2GRAY)
        
        #analiza klatki z uzyciem sieci neuronowych
        result = self.tensorflowProcessor.getBallPosition(tracker_frame)
        result = np.round(result * self.ballTracker_size).astype("int")
        
        self.ballTracker_result[0] = self.ballTracker_pos[0] + result[0]
        self.ballTracker_result[1] = self.ballTracker_pos[1] + result[1]
        
        #zaznaczanie wizualne pozycji kulki
        #cv2.circle(self.frame_original, tuple(self.ballTracker_result), 1, (0, 0, 255), -1)
    
    #znajduje pozycje krawedzi plyty
    def FindBoardCorners(self):
        corners = [(0, 0), (0, 0), (0, 0), (0, 0)]
        corner_detection_area_pixels = (round(self.corner_detecton_area[0] * self.camera_resolution[0]),
                                       round(self.corner_detecton_area[1] * self.camera_resolution[1]),
                                       round(self.corner_detecton_area[2] * self.camera_resolution[0]),
                                       round(self.corner_detecton_area[3] * self.camera_resolution[1]))
        for i in range(4):
            flipX = False
            flipY = False
            detectionArea = list(corner_detection_area_pixels)    #domyslnie lewy gorny
            if i == 1 or i == 2:
                detectionArea[0] = self.camera_resolution[0] - detectionArea[0] - detectionArea[2]
                flipX = True
            if i == 3 or i == 2:
                detectionArea[1] = self.camera_resolution[1] - detectionArea[1] - detectionArea[3]
                flipY = True
                
            rect = (detectionArea[0], detectionArea[1], detectionArea[0] + detectionArea[2], detectionArea[1] + detectionArea[3])
            #cv2.rectangle(self.frame_debug, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 1);
        
            img = self.frame_original[rect[1]:rect[3], rect[0]:rect[2]]
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = cv2.resize(img, (40, 40), interpolation=cv2.INTER_NEAREST)
            
            if flipX and flipY: img = cv2.flip(img, -1)
            elif flipX: img = cv2.flip(img, 1)
            elif flipY: img = cv2.flip(img, 0)
            #cv2.imshow("Corner " + str(i), img)
            
            result = self.tensorflowProcessor.getCornerPosition(img)
            corner = np.round(result * 40.0).astype("int")
            
            if flipX and flipY: corners[i] = (40 - corner[0] + detectionArea[0], 40 - corner[1] + detectionArea[1])
            elif flipX: corners[i] = (40 - corner[0] + detectionArea[0], corner[1] + detectionArea[1])
            elif flipY: corners[i] = (corner[0] + detectionArea[0], 40 - corner[1] + detectionArea[1])
            else: corners[i] = (corner[0] + detectionArea[0], corner[1] + detectionArea[1])
            #cv2.circle(self.frame_debug, corners[i], 1, (0, 0, 255), 1)

        return np.array(corners, np.int32)

    #zmienia perspektywe obrazu z kamery tak, aby niewidoczne bylo przechylenie plyty
    def ChangePerspective(self):
        pts = np.array(self.corners, np.float32)
        pts2 = np.float32([[0,0],[self.detection_image_resolution[0],0],[self.detection_image_resolution[0], self.detection_image_resolution[1]], [0,self.detection_image_resolution[1]]])

        M = cv2.getPerspectiveTransform(pts, pts2)
        self.frame_original = cv2.warpPerspective(self.frame_original, M, self.detection_image_resolution)