import ImageProcessingModule as IPM
import ServoControllerModule as SCM
import PIDControllerModule as PIDCM
import DataLoggerModule as DLM

from time import sleep
import time
import pygame
import math
import MathModule as MM

#initial setup
imageProcessor = IPM.ImageProcessor()
servoController = SCM.ServoController()
dataLogger = DLM.DataLogger()
pidController = PIDCM.PIDController()
pidController.servo_pos_limit = servoController.servo_pos_limit

pygame.init()
pygame.display.set_mode((100, 100))

#ball recognition process start
imageProcessor.StartProcessing()
sleep(2)    #time for camera to startup

targetDeltaTime = 1.0 / 40.0    #control loop refresh delta
updatedTime = 0.0
servoUpdateDeltaTime = 1.0 / 60
servoUpdatedTime = 0.0

ball_position_actual = (0.0, 0.0)
ball_position_previous = (0.0, 0.0)

#jak dlugo ma wykonywany ma byc program
duration = 20000
timeout = time.time() + duration
ball_just_found = True    #czy kulka dopiero zostala znaleziona i nalezy zresetowac predkosc?

while time.time() <= timeout:
    timeStart = time.perf_counter()
    
    if timeStart - updatedTime >= targetDeltaTime:
        updatedTime = time.perf_counter()
        
        ball_position_actual = imageProcessor.getBallPosition()
        if ball_position_actual[0] >= 0:
            pidController.setActualValue(ball_position_actual)
            
        pidController.update(targetDeltaTime)
        ball_position_previous = ball_position_actual
        
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_g:
                    pidController.increaseKP()
                    
                elif event.key == pygame.K_b:
                    pidController.decreaseKP()
                    
                elif event.key == pygame.K_h:
                    pidController.increaseKI()
                    
                elif event.key == pygame.K_n:
                    pidController.decreaseKI()
                    
                elif event.key == pygame.K_j:
                    pidController.increaseKD()
                    
                elif event.key == pygame.K_m:
                    pidController.decreaseKD()
        
        servoController.moveServo(0, round(pidController.x_servo))
        servoController.moveServo(1, -round(pidController.y_servo))
        
        #dodawanie wpisow do DataLog'u
        dataLogger.addRecord("timestamp", time.perf_counter())
        dataLogger.addRecord("ball_pos_x", ball_position_actual[0])
        dataLogger.addRecord("ball_pos_y", ball_position_actual[1])
        #dataLogger.addRecord("ball_vel_x", pidController.ball_velocity_actual[0])
        #dataLogger.addRecord("ball_vel_y", pidController.ball_velocity_actual[1])
        dataLogger.addRecord("KP", pidController.KP)
        dataLogger.addRecord("KI", pidController.KI)
        dataLogger.addRecord("KD", pidController.KD)
        dataLogger.addRecord("error_x", pidController.x_error)
        dataLogger.addRecord("error_y", pidController.y_error)
        dataLogger.addRecord("error_prev_x", pidController.x_prev_error)
        dataLogger.addRecord("error_prev_y", pidController.y_prev_error)
        dataLogger.addRecord("error_sum_x", pidController.x_error_sum)
        dataLogger.addRecord("error_sum_y", pidController.y_error_sum)
        dataLogger.addRecord("derivative_x", pidController.x_derivative)
        dataLogger.addRecord("derivative_y", pidController.y_derivative)
        dataLogger.addRecord("servo_actual_x", servoController.servo_actual_pos[0])
        dataLogger.addRecord("servo_actual_y", servoController.servo_actual_pos[1])
        dataLogger.addRecord("servo_target_x", servoController.servo_target_pos[0])
        dataLogger.addRecord("servo_target_y", servoController.servo_target_pos[1])
        #dataLogger.saveRecord()
        
    if time.perf_counter() - servoUpdatedTime >= servoUpdateDeltaTime:
        servoController.update(time.perf_counter() - servoUpdatedTime)
        servoUpdatedTime = time.perf_counter()

dataLogger.saveToFile("BallanceDataLog")