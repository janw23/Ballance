if __name__ == '__main__':
    simulationMode = True    #czy uruchomic program w trybie symulacji? wymaga rowniez zmiany w ServoControllerModule.py oraz w ImageProcessingModule.py

    import ImageProcessingModule as IPM
    import ServoControllerModule as SCM
    import PIDControllerModule as PIDCM
    import DataLoggerModule as DLM
    import PathPlannerModule as PPM
    
    from time import sleep
    import time
    import pygame
    import math
    import MathModule as MM

    #wykonanie wstepnych czynnosci
    if simulationMode:
        import SimulationCommunicatorModule as SimCM
        simulationCommunicator = SimCM.SimulationCommunicator()
    else: simulationCommunicator = None
    
    imageProcessor = IPM.ImageProcessor(simulationCommunicator)
    servoController = SCM.ServoController()
    pathPlanner = PPM.PathPlanner()
        
    dataLogger = DLM.DataLogger()
    pidController = PIDCM.PIDController()
    pidController.servo_pos_limit = servoController.servo_pos_limit

    pygame.init()
    pygame.display.set_mode((100, 100))

    #roizpoczynanie procesu wykrywania kulki
    if simulationMode: simulationCommunicator.StartProcessing()
    imageProcessor.StartProcessing()
    pathPlanner.startProcessing(imageProcessor.obstacle_map)

    targetDeltaTime = 1.0 / 40.0    #czas jednej iteracji programu sterujacego
    updatedTime = 0.0
    servoUpdateDeltaTime = 1.0 / 60 #czas odswiezania pozycji serw
    servoUpdatedTime = 0.0

    ball_position_actual = (0.0, 0.0)
    ball_position_previous = (0.0, 0.0)

    #parametry trajektorii kulki
    angle = 0.0
    angleSpeed = 0.9
    angleRadius = 0.25
    angleRadiusFactor = 0.0
    path_targets = [(0.18, 0.18), (0.82, 0.82)]
    path_target_index = 0
    targetPos = path_targets[path_target_index]
    moveSpeed = 0.05
    movementMode = 0
    modeChangeTimeDelta = 25 #czas po jakim zmieniana jest trajektoria kulki
    modeChangeTimer = 0.0

    #jak dlugo wykonywany ma byc program
    duration = 10000
    timeout = time.time() + duration
    ball_just_found = True    #czy kulka dopiero zostala znaleziona i nalezy zresetowac predkosc?

    #glowna petla programu
    while time.time() <= timeout:
        timeStart = time.perf_counter()
        
        #oczekiwanie na odpowiedni moment do wykonania programu sterujacego
        if timeStart - updatedTime >= targetDeltaTime:
            updatedTime = time.perf_counter()
            
            #pobranie pozycji kulki
            ball_position_actual = imageProcessor.getBallPosition()
            if ball_position_actual[0] >= 0: pidController.setActualValue(ball_position_actual)
            else: pidController.setActualValue(pidController.value_target)
                
            #aktualizacja kontrolera PID
            pidController.update(targetDeltaTime)
            ball_position_previous = ball_position_actual
            
            #aktualizacja pozycji kulki w pathplannerze
            pathPlanner.setBallPosition(ball_position_actual)
            pidController.setTargetValue(pathPlanner.getPathTarget())
            
            #przechodzenie do kolejnego waypoint'a
            if MM.sqrMagnitude(ball_position_actual[0] - targetPos[0], ball_position_actual[1] - targetPos[1]) < 0.01:
                path_target_index = (path_target_index + 1) % len(path_targets)
                targetPos = path_targets[path_target_index]
                pathPlanner.setTargetPosition(targetPos)
            #print(str(pidController.value_target))
            
            #obslugiwanie wejscia z klawiatury
            killLoop = False
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
                        
                    elif event.key == pygame.K_q:
                        killLoop = True
                        
                    elif event.key == pygame.K_UP:
                        targetPos[1] -= moveSpeed
                        
                    elif event.key == pygame.K_DOWN:
                        targetPos[1] += moveSpeed
                        
                    elif event.key == pygame.K_RIGHT:
                        targetPos[0] += moveSpeed
                        
                    elif event.key == pygame.K_LEFT:
                        targetPos[0] -= moveSpeed
                        
                    elif event.key == pygame.K_p:
                        angleSpeed += 0.1
                        print("angleSpeed = " + str(angleSpeed))
                        
                    elif event.key == pygame.K_o:
                        angleSpeed -= 0.1
                        print("angleSpeed = " + str(angleSpeed))
                        
            if killLoop:
                break
            
            #ustawianie nowych pozycji serw
            servoController.moveServo(0, round(pidController.x_servo))
            servoController.moveServo(1, -round(pidController.y_servo))
            
            #dostepne trajektorie ruchu kulki
            if False:
                if movementMode == 0:    #ksztalt osemki
                    targetPos[0] = math.sin(angle)
                    targetPos[1] = math.sin(2.0 * angle)
                elif movementMode == 1:  #ksztalt okregu
                    targetPos[0] = math.sin(angle)
                    targetPos[1] = math.cos(angle)
                elif movementMode == 2:   #ksztalt paraboli
                    targetPos[0] = math.sin(angle)
                    targetPos[1] = math.cos(2.0 * angle)
                elif movementMode == 3:   #ksztalt litery S
                    targetPos[0] = math.sin(angle)
                    targetPos[1] = math.sin(2.0 * angle)
                    if angle > 2:
                        angleSpeed = -angleSpeed
                        angle = 2
                    elif angle < -2:
                        angleSpeed = -angleSpeed
                        angle = -2
                    
            #targetPos[0] = 0.5 + angleRadiusFactor * angleRadius * targetPos[0]
            #targetPos[1] = 0.5 + angleRadiusFactor * angleRadius * targetPos[1]
            #ustawianie docelowej pozycji kulki
            #pidController.setTargetValue(targetPos[0], targetPos[1])
            #pathPlanner.setTargetPosition(tuple(targetPos))
            angle += angleSpeed * targetDeltaTime
            angleRadiusFactor += 0.25 * targetDeltaTime
            angleRadiusFactor = min(angleRadiusFactor, 1.0)
            
            modeChangeTimer += targetDeltaTime
            if modeChangeTimer >= modeChangeTimeDelta:
                modeChangeTimer = 0.0
                angleRadiusFactor = 0.0
                movementMode += 1
                movementMode = movementMode % 4
            
            #dodawanie wpisow do DataLog'u
            if False:
                path_target = pathPlanner.getPathTarget()
                dataLogger.addRecord("timestamp", time.perf_counter())
                dataLogger.addRecord("ball_pos_x", ball_position_actual[0])
                dataLogger.addRecord("ball_pos_y", ball_position_actual[1])
                dataLogger.addRecord("target_pos_x", path_target[0])
                dataLogger.addRecord("target_pos_y", path_target[1])
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
            
        #oczekiwanie na odpowiedni moment do aktualizacji serw
        if time.perf_counter() - servoUpdatedTime >= servoUpdateDeltaTime:
            servoController.update(time.perf_counter() - servoUpdatedTime)
            servoUpdatedTime = time.perf_counter()
            
            if simulationMode:
                simulationCommunicator.moveServos(servoController.servo_actual_pos)
                
        sleep(0.004) #4 milisekundy na odpoczynek :)
            
    print("Stopping program")
    #dataLogger.saveToFile("BallanceDataLog")
    if simulationMode: simulationCommunicator.StopProcessing()
    else: imageProcessor.StopProcessing()
    pathPlanner.stopProcessing()