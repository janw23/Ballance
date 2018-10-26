from picamera.array import PiRGBArray
from picamera import PiCamera
import time

from multiprocessing import Process
from multiprocessing import Value
 
class ImageProcessor(object):
    
    #stale
    camera_resolution = (100, 100)
    camera_framerate = 30
    
    def InitializeCamera(self):    #inicjalizacja kamery
        self.camera = PiCamera()
        self.camera.resolution = ImageProcessor.camera_resolution
        self.camera.framerate = ImageProcessor.camera_framerate
        self.rawCapture = PiRGBArray(self.camera, size=ImageProcessor.camera_resolution)    #przygotowanie miejsca na przechowanie obrazu z kamery w surowej formie
        
        time.sleep(0.1)    #czas dla kamery na ustawienie parametrow
    
    
    def __init__(self):
        ImageProcessor.InitializeCamera(self)
        
        
    def StartProcessing(self):   #uruchamia proces przetwarzajacy obraz
        #wartosci-rezultaty przetwarzania obrazu
        self.result_x = Value('d', 0.0)
        self.result_y = Value('d', 0.0)
        
        self.process = Process(target=ImageProcessor.ProcessImage, args=(self,))
        self.process.daemon = True
        self.process.start()
        
        
    def StopProcessing(self):    #wydaje polecenie do zatrzymania przetwarzania obrazu
        self.process.terminate()
        
        
    def ProcessImage(self):    #przetwarza obraz pobierajac klatke z kamery i wykonujac na niej operacje
        
        for self.frame in self.camera.capture_continuous(self.rawCapture, format='rgb', use_video_port=True):
            self.capturedImage = self.frame.array #zapisywanie otrzymanego zdjecia jako tablicy
            
            self.result = ImageProcessor.FindBall(self.capturedImage)   #znajdowanie kulki na obrazie i zwracanie rezultatu
            self.result_x = self.result[0]    #ustawianie odpowiedzi w wartosciach dzielonych miedzy procesami
            self.result_y = self.result[1]
            
            print(str(result[0]))
            
            self.rawCapture.truncate(0)
            
            
    def FindBall(image):
        return (5.76, 3.865)
 