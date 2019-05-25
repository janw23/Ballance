from __future__ import division
simulationMode = False

if not simulationMode:
    import sys
    sys.path.append('/home/pi/Adafruit_Python_PCA9685/')
    import Adafruit_PCA9685
    
import math
import MathModule as MM

#program kontrolujacy ruch serw
class ServoController:
    
    #parametry serw
    servo_pulse_neutral = (388, 379)    #wartosci pwm dla pozycji neutralnych serw
    servo_pulse_range = (100, 100)      #zakres wartosci sygnalu pwm dla ruchu serw
    servo_pos_limit = (800, 800)    #ograniczenia wychylen serw (w skali od 0 do 1000)
    servo_movement_speed = (6000, 6000)    #szybkosci ruchu serw
    
    def __init__(self):
        if not simulationMode:
            self.pwm = Adafruit_PCA9685.PCA9685()  #laczenie sie z plytka sterujaca serwami
            self.pwm.set_pwm_freq(60)
        
        #zmienne wartosci
        self.servo_actual_pos = [0, 0]    #aktualna pozycja serwa
        self.servo_target_pos = [0, 0]    #docelowa pozycja serwa
        
        self.update(0)   #aplikowanie domyslnych ustawien serw

    #wydaje polecenie poruszenia serwem na kanale 'channel' na pozycje 'pos' (w skali od -1000 do 1000)
    def moveServo(self, channel, pos):
        self.servo_target_pos[channel] = MM.clamp(pos, -ServoController.servo_pos_limit[channel], ServoController.servo_pos_limit[channel])

    #aktualizuje pozycje serw
    def update(self, deltaTime):
        for i in range(2): #tylko 2 serwa
            
            movement_dir = MM.sign(self.servo_target_pos[i] - self.servo_actual_pos[i])
            self.servo_actual_pos[i] += ServoController.servo_movement_speed[i] * movement_dir * deltaTime
            
            if movement_dir > 0: self.servo_actual_pos[i] = min(self.servo_actual_pos[i], self.servo_target_pos[i])
            elif movement_dir < 0: self.servo_actual_pos[i] = max(self.servo_actual_pos[i], self.servo_target_pos[i])
                
            if not simulationMode:
                pos = round(ServoController.servo_pulse_neutral[i] + ServoController.servo_pulse_range[i] * self.servo_actual_pos[i] / 1000)
                self.pwm.set_pwm(i, 0, pos)
            else:
                self.servo_actual_pos[i] = round(self.servo_actual_pos[i])
