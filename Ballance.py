#biblioteka potrzebna do serw
from __future__ import division

#importowanie potrzebnych bibliotek do sterowania serwami
import sys
sys.path.append('/home/pi/Adafruit_Python_PCA9685/')

#importowanie biblioteki do serw
import Adafruit_PCA9685

#importowanie dodatkowych bibliotek pomocniczych
import io
import time
import math
import threading
from PIL import Image
import numpy as np

#biblioteka do obliczen rownoleglych
from multiprocessing import Pool
from multiprocessing import Process
from multiprocessing import Array, Value


#importowanie bibliotek kamery
from picamera import PiCamera

#biblioteki potrzebne do czytania wejscia z klawiatury
import pygame


#################################
#POMOCNICZE FUNKCJE MATEMATYCZNE#
#################################

def sign(num):
    if num > 0:
        return 1.0
    
    elif num < 0:
        return -1.0
    
    return 0.0

##########################################
#USTAWIENIA CZYTANIA WEJSCIA Z KLAWIATURY#
##########################################
pygame.init()

pygame.display.set_mode((100, 100))

key_pressed = [False, False, False, False]

def DetectInput():
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RIGHT:
                key_pressed[0] = True
            
            if event.key == pygame.K_LEFT:
                key_pressed[1] = True
                
            if event.key == pygame.K_UP:
                key_pressed[2] = True
                
            if event.key == pygame.K_DOWN:
                key_pressed[3] = True
                
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_RIGHT:
                key_pressed[0] = False
            
            if event.key == pygame.K_LEFT:
                key_pressed[1] = False
                
            if event.key == pygame.K_UP:
                key_pressed[2] = False
                
            if event.key == pygame.K_DOWN:
                key_pressed[3] = False

####################################################
#USTAWIENIA POCZATKOWE DOTYCZOCE STEROWANIA SERWAMI#
####################################################

#inicjalizacja lacza pwm
pwm = Adafruit_PCA9685.PCA9685()

#ustawianie odpowiedniej czestotliwosci pwm
pwm.set_pwm_freq(60)

class PIDController:
    refresh_time_delta = 1.0/30.0
    
    def UpdatePID(self):
        self.x_error = self.ball_actual_velocity[0] - self.ball_target_velocity[0]
        self.y_error = self.ball_actual_velocity[1] - self.ball_target_velocity[1]
        
        self.x_servo += (self.x_error * self.KP.value) + (self.x_prev_error * self.KD.value) + (self.x_error_sum * self.KI.value)
        self.y_servo += (self.y_error * self.KP.value) + (self.y_prev_error * self.KD.value) + (self.y_error_sum * self.KI.value)
        
        self.x_servo = min(250.0, max(-250.0, self.x_servo))
        self.y_servo = min(250.0, max(-250.0, self.y_servo))
        
        servoUpdater.moveServo(0, int(self.x_servo))
        servoUpdater.moveServo(1, -int(self.y_servo))
        
        self.x_prev_error = self.x_error
        self.y_prev_error = self.y_error
        
        self.x_error_sum += self.x_error
        self.y_error_sum += self.y_error
        
        self.x_error_sum *= 0.92
        self.y_error_sum *= 0.92
       
        #print(PIDController.coef_P * (self.ball_target_velocity[0] - self.ball_actual_velocity[0]))
    
    def dotask(self):
        # This method runs in a separate thread
        lastTime = time.time()
        currentTime = lastTime
        
        while not self.terminated:
            lastTime = time.time()
            PIDController.UpdatePID(self)
            currentTime = time.time()
            
            if currentTime - lastTime < PIDController.refresh_time_delta:
                time.sleep(PIDController.refresh_time_delta- currentTime + lastTime)
            else:
                print('PIDController update cycle took more than specified')
    
    def __init__(self, servoupdater):
        #super(ServoUpdater, self).__init__()
        self.ball_target_velocity = Array('d', [0.0, 0.0])
        self.ball_actual_velocity = Array('d', [0.0, 0.0])
        
        self.KP = Value('d', 142.0)
        self.KD = Value('d', 0.0)
        self.KI = Value('d', 0.0)
        
        self.x_servo = 0.0
        self.y_servo = 0.0
        
        self.x_error = 0.0
        self.y_error = 0.0
        
        self.x_prev_error = 0.0
        self.y_prev_error = 0.0
        
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0
                               
        self.terminated = False
        
        self.servoUpdater = servoupdater
        
        self.controllerProcess = Process(target=self.dotask)
        self.controllerProcess.daemon = True
        self.controllerProcess.start()
    

#klasa zarzadzajaca pozycja serw (w funkcji moveServo podaje sie zadana pozycje serwa i ta klasa steruje tym serwem tak, zeby osiagnal te pozycje)        
class ServoUpdater:
    #ustawienia serw (tablice z wartosciami parametrow dla poszczegolnych kanalow)
    servo_pulse_neutral = (385, 396)
    servo_pulse_range = (100, 100)

    #aktualne i docelowe pozycje serw
    servo_actual_pos = [0, 0]
    #ogranicznik wychylenia serw w skali od 0 do 1000
    servo_pos_limit = (250, 250)
    #servo_target_pos = [0, 0]

    #szybkosc ruchu danego serwa
    servo_movement_speed = (5000, 5000)
    
    #czas pomiedzy odswiezeniem pozycji serw
    servo_refresh_time_delta = 1.0/60.0


    #porusza serwem na kanale 'channel' na pozycje 'pos' (w skali od -1000 do 1000)
    def moveServo(self, channel, pos):
        self.servo_target_pos[channel] = max(-ServoUpdater.servo_pos_limit[channel], min(ServoUpdater.servo_pos_limit[channel], pos))

    #aktualizuje pozycje serw
    def updateServoPositions(self, deltaTime):
        for i in range(2): #tylko 2 serwa
            movement_dir = sign(self.servo_target_pos[i] - self.servo_actual_pos[i])
        
            self.servo_actual_pos[i] += self.servo_movement_speed[i] * movement_dir * deltaTime
            
            if movement_dir > 0:
                self.servo_actual_pos[i] = min(self.servo_actual_pos[i], self.servo_target_pos[i])
                
            elif movement_dir < 0:
                self.servo_actual_pos[i] = max(self.servo_actual_pos[i], self.servo_target_pos[i])
                
                
            pos = int(self.servo_pulse_neutral[i] + self.servo_pulse_range[i] * self.servo_actual_pos[i] / 1000)
            pwm.set_pwm(i, 0, pos)
            
    def dotask(self):
        # This method runs in a separate thread
        lastTime = time.time()
        currentTime = lastTime
        
        while not self.terminated:
            lastTime = time.time()
            ServoUpdater.updateServoPositions(self, ServoUpdater.servo_refresh_time_delta)
            currentTime = time.time()
            
            if currentTime - lastTime < ServoUpdater.servo_refresh_time_delta:
                time.sleep(ServoUpdater.servo_refresh_time_delta - currentTime + lastTime)
            else:
                print('Servo update cycle took more than specified')
    
    def __init__(self):
        #super(ServoUpdater, self).__init__()
        self.servo_target_pos = Array('i', [0, 0])
        self.terminated = False
        self.updaterProcess = Process(target=self.dotask)
        self.updaterProcess.daemon = True
        self.updaterProcess.start()
    
    
##################
#WYKRYWANIE KULKI#
##################
                
#NAPISAC PRZETWARZANIE OBRAZU Z UZYCIEM BIBLIOTEKI MULTIPROCESSING

poolSize = 3
colorThreshold = 90

#przetwarza otrzymany obraz i zwraca liczbe wykrytych pixeli i ich sume pozycji
def ProcessImage(image, threadNumber):
    pixels = image.load()
    
    skip = 7
    pixelCount = 0
    
    average = 0.0
    
    y = 0
    x = 0
    
    while x < image.width:
        while y < image.height:
            pixel = pixels[x, y]
            
            average += (pixel[0] + pixel[1] + pixel[2]) / 3
            pixelCount += 1
            
            y += skip
        
        y = 0
        x += skip
        
    average /= pixelCount
    average += colorThreshold
    
    x = 0
    y = image.height // poolSize * threadNumber
    stopY = y + image.height // poolSize
    
    if threadNumber == poolSize - 1:
        stopY = image.height
        
    pixelCount = 0
    pixelSumPos = [0, 0]
        
    while y < stopY:
        for x in range(image.width):
            pixel = pixels[x, y]
            if (pixel[0] + pixel[1] + pixel[2]) / 3 >= average:
                pixelSumPos[0] += x
                pixelSumPos[1] += y
                
                pixelCount += 1
        y += 1
        
    return (pixelSumPos, pixelCount)

    
def ProcessImage_unpacked(args):
    return ProcessImage(*args)
    
        
#przetwarza klatki otrzymane z kamery
class ImageProcessor(threading.Thread):
    def __init__(self, owner):
        super(ImageProcessor, self).__init__()
        self.daemon = True
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        
        self.lastTime = time.time()
        self.position_previous = [0.0, 0.0]
        
        #ustawienia dotyczace procesow przetwarzajacych obraz
        self.processPool = Pool(processes=poolSize)
        self.processPool.daemon = True
        
        self.start()
        
        

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    self.stream.seek(0)
                    # Read the image and do some processing on it
                    
                    #startTime = time.time()
                    capturedImage = Image.open(self.stream)
                    
                    poolResult = self.processPool.imap_unordered(ProcessImage_unpacked, ((capturedImage, 0), (capturedImage, 1), (capturedImage, 2), ))
                    
                    #print(poolResult)
                    
                    pixelCount = 0
                    pixelSumPos = [0.0, 0.0]
                    
                    for result in poolResult:
                        pixelSumPos[0] += result[0][0]
                        pixelSumPos[1] += result[0][1]
                        
                        pixelCount += result[1]
                    
                    if pixelCount > 0:
                        self.owner.result_ball_position[0] = pixelSumPos[0] / pixelCount / capturedImage.width
                        self.owner.result_ball_position[1] = pixelSumPos[1] / pixelCount / capturedImage.height
                        
                    self.owner.result_ball_velocity = [self.position_previous[0] - self.owner.result_ball_position[0],
                                                       self.position_previous[1] - self.owner.result_ball_position[1]]
                    
                    currentTime = time.time()
                    self.owner.result_ball_velocity[0] /= currentTime - self.lastTime
                    self.owner.result_ball_velocity[1] /= currentTime - self.lastTime
                    
                    self.position_previous = [self.owner.result_ball_position[0], self.owner.result_ball_position[1]]
                    self.lastTime = time.time()
                    
                    #print(self.owner.result_ball_velocity)
                    
                    
                        
                    #print(self.owner.result_ball_position)
                    
                    #print(pixelCount)
                    
                    #capturedImage.putpixel(self.owner.result_ball_position, (255, 0, 0))
                    #capturedImage.save('out.bmp')
                    
                    
                    
                    #print('Processing took ' + str(time.time() - startTime))
                    
                    # Set done to True if you want the script to terminate
                    # at some point
                    #self.owner.done=True
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)

#przetwarza klatki otrzymanej z kamery
class ProcessOutput(object):
    def __init__(self):
        #znaleziona pozycja kulki na zdjeciu
        self.result_ball_position = [0.0, 0.0]
        self.result_ball_velocity = [0.0, 0.0]
        
        self.done = False
        # Construct a pool of specified count of image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.pool = [ImageProcessor(self) for i in range(4)]
        self.processor = None
        print('starting image processing')

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    print('The frame was skipped')
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True
            proc.join()

    
########################################
#USTAWIENIA POCZATKOWE DOTYCZOCE KAMERY#
########################################
    
camera = PiCamera()

#ustawianie rozdzielczosci kamery
camera.resolution = (70, 70)
camera.framerate = 20

#ustawianie parametrow ruchu serwami
delay = 1.0/30.0
movement_range = 150

##################
#PROGRAM WLASCIWY#
##################
if __name__=='__main__': 

    camera.start_preview(fullscreen=False, window=(200, 200, 400, 400))
    
    #watek ktory aktualizuje pozycje serw
    servoUpdater = ServoUpdater()
    
    #watek, ktory kontroluje wychylenia serw na podstawie predkosci kulki
    pidController = PIDController(servoUpdater)
    
    #kamera nagrywa i kolejne otrzymane klatki sa przetwarzane
    time.sleep(2)
    output = ProcessOutput()
    camera.start_recording(output, format='mjpeg')
    
    while not output.done:
        camera.wait_recording(0.01)
        
        pidController.ball_actual_velocity[0] = output.result_ball_velocity[0]
        pidController.ball_actual_velocity[1] = output.result_ball_velocity[1]
        
        DetectInput() #sprawdzanie wejscia z klawiatury
        #updateServoPositions()
        
        #lewo i prawo
        if key_pressed[0]:
            pidController
            pidController.KP.value += 1
            
            print('KP = ' + str(pidController.KP))
            
        elif key_pressed[1]:
            pidController.KP.value -= 1
            print('KP = ' + str(pidController.KP))
            
        #gora i dol
        if key_pressed[2]:
            pidController.KD.value += 1
            print('KD = ' + str(pidController.KD))
            
        elif key_pressed[3]:
            pidController.KD.value -= 1
            print('KD = ' + str(pidController.KD))
            
        
        #print(output.result_ball_position)
        
    camera.stop_recording()

    while False:
        time.sleep(delay)
        DetectInput() #sprawdzanie wejscia z klawiatury
        #updateServoPositions()
        
        #lewo i prawo
        if key_pressed[0]:
            servoUpdater.moveServo(0, movement_range)
            
        elif key_pressed[1]:
            servoUpdater.moveServo(0, -movement_range)
            
        else:
            servoUpdater.moveServo(0, 0)
            
        #gora i dol
        if key_pressed[2]:
            servoUpdater.moveServo(1, movement_range)
            
        elif key_pressed[3]:
            servoUpdater.moveServo(1, -movement_range)
            
        else:
            servoUpdater.moveServo(1, 0)

