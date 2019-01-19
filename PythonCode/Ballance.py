import ImageProcessingModule as IPM
import ServoControllerModule as SCM
import PIDControllerModule as PIDCM
import DataLoggerModule as DLM

from time import sleep
import time
import pygame
import math
import MathModule as MM

#TEMPORARY STUFF
#import matplotlib.pyplot as plt
#import matplotlib.animation as animation

#fig = plt.figure()
#ax1 = fig.add_subplot(1, 1, 1)

#list_velocity = []
#list_time = []

#def animate(i):
#    ax1.clear()
#    ax1.plot(list_time, list_velocity)
    
#ani = animation.FuncAnimation(fig, animate, interval=1000)
#plt.show()
#fig.show()
#fig.canvas.draw()

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

ball_position_actual = (0.0, 0.0)
ball_position_previous = (0.0, 0.0)


key_pressed = [False, False, False, False]

a = 0
#jak dlugo ma wykonywany ma byc program
duration = 30
timeout = time.time() + duration
ball_just_found = True    #czy kulka dopiero zostala znaleziona i nalezy zresetowac predkosc?

while time.time() <= timeout:
    timeStart = time.perf_counter()
    
    if True:
        ball_position_actual = imageProcessor.getBallPosition()
        
        if ball_position_actual[0] >= 0:    #jesli wartosc jest ujemna, to znaczy, ze kulka nie zostala wykryta
            if ball_just_found:    #jesli kulka zostala dopiero znaleziona
                ball_position_previous = ball_position_actual
                ball_just_found = False
                
            pidController.setActualVelocity((ball_position_actual[0] - ball_position_previous[0]) / targetDeltaTime, (ball_position_actual[1] - ball_position_previous[1]) / targetDeltaTime)
        else:
            ball_just_found = True
            
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
    
    #animate graph
    #vel = math.sqrt(math.pow(pidController.ball_actual_velocity[0], 2) + math.pow(pidController.ball_actual_velocity[1], 2))
    #print(vel)
    #list_velocity.append(vel)
    #list_time.append(a)
    
    #if len(list_velocity) == 100:
    #    list_velocity.pop(0)
    #    list_time.pop(0)
        
    #ax1.clear()
    #ax1.plot(list_time, list_velocity)
    #plt.plot(list_time, list_velocity)
    #fig.canvas.draw()
    
    #a += 1
    
    #print(pidController.ball_actual_velocity)
    
    servoController.moveServo(0, int(pidController.x_servo))
    servoController.moveServo(1, -int(pidController.y_servo))
    servoController.update(targetDeltaTime)
        
    #dodawanie wpisow do DataLog'u
    dataLogger.addRecord("timestamp", time.perf_counter())
    dataLogger.addRecord("ball_pos_x", ball_position_actual[0])
    dataLogger.addRecord("ball_pos_y", ball_position_actual[1])
    dataLogger.addRecord("ball_vel_x", pidController.ball_velocity_actual[0])
    dataLogger.addRecord("ball_vel_y", pidController.ball_velocity_actual[1])
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
    dataLogger.saveRecord()
    
    sleep(max(targetDeltaTime - time.perf_counter() + timeStart, 0))



imageProcessor.StopProcessing()
dataLogger.saveToFile("BallanceDataLog")