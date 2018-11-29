import ImageProcessingModule as IPM
import ServoControllerModule as SCM
import PIDControllerModule as PIDCM

from time import sleep
import time
import pygame

imageProcessor = IPM.ImageProcessor()
servoController = SCM.ServoController()
pidController = PIDCM.PIDController()
pidController.servo_pos_limit = servoController.servo_pos_limit

pygame.init()
pygame.display.set_mode((100, 100))

imageProcessor.StartProcessing()
sleep(2)

targetDeltaTime = 1.0 / 40.0

ball_position_actual = (0.0, 0.0)
ball_position_previous = (0.0, 0.0)

key_pressed = [False, False, False, False]

while True:
    timeStart = time.perf_counter()
    
    if True:
        ball_position_actual = imageProcessor.getBallPosition()
        pidController.ball_actual_velocity = ((ball_position_actual[0] - ball_position_previous[0]) / targetDeltaTime, (ball_position_actual[1] - ball_position_previous[1]) / targetDeltaTime)
        pidController.update(targetDeltaTime)
        ball_position_previous = ball_position_actual
    else:
        r = 300
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
                   
        if key_pressed[0]:
            servoController.moveServo(0, r)
        elif key_pressed[1]:
            servoController.moveServo(0, -r)
        else:
            servoController.moveServo(0, 0)
            
        if key_pressed[2]:
            servoController.moveServo(1, r)
        elif key_pressed[3]:
            servoController.moveServo(1, -r)
        else:
            servoController.moveServo(1, 0)
    
    #print(pidController.ball_actual_velocity)
    
    servoController.moveServo(0, int(pidController.x_servo))
    servoController.moveServo(1, -int(pidController.y_servo))
    servoController.update(targetDeltaTime)
    
    sleep(targetDeltaTime - time.perf_counter() + timeStart)



imageProcessor.StopProcessing()