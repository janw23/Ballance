from __future__ import division
import sys
sys.path.append('/home/pi/Adafruit_Python_PCA9685/')
import Adafruit_PCA9685
import math
import MathModule as MM

class ServoController:
    
    #stale wartosci
    servo_pulse_neutral = (380, 370)    #wartosci pwm dla pozycji neutralnych serw
    servo_pulse_range = (100, 100)      #zakres wartosci sygnalu pwm dla ruchu serw
    servo_pos_limit = (300, 300)    #ograniczenia wychylen serw (w skali od 0 do 1000)
    servo_movement_speed = (4000, 4000)    #szybkosci ruchu serw
    
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()  #laczenie sie z plytka sterujaca serwami
        self.pwm.set_pwm_freq(60)
        
        #zmienne wartosci
        self.servo_actual_pos = [0, 0]    #aktualna pozycja serwa
        self.servo_target_pos = [0, 0]    #docelowa pozycja serwa
        
        #aplikowanie domyslnych ustawien serw
        self.update(0)

    def moveServo(self, channel, pos):   #wydaje polecenie poruszenia serwem na kanale 'channel' na pozycje 'pos' (w skali od -1000 do 1000)
        self.servo_target_pos[channel] = max(-ServoController.servo_pos_limit[channel], min(ServoController.servo_pos_limit[channel], pos))

    def update(self, deltaTime):    #aktualizowanie pozycji serw
        #normalizacja wektora docelowego wychylenia
        #magnitude = math.sqrt(self.servo_target_pos[0] * self.servo_actual_pos[0] + self.servo_target_pos[1] * self.servo_actual_pos[1])
        #if magnitude != 0:
        #    self.servo_target_pos[0] = min(magnitude, ServoController.servo_pos_limit[0]) * self.servo_target_pos[0] / magnitude
        #    self.servo_target_pos[1] = min(magnitude, ServoController.servo_pos_limit[1]) * self.servo_target_pos[1] / magnitude
        
        for i in range(2): #tylko 2 serwa
            
            movement_dir = MM.sign(self.servo_target_pos[i] - self.servo_actual_pos[i])
            self.servo_actual_pos[i] += ServoController.servo_movement_speed[i] * movement_dir * deltaTime
            
            if movement_dir > 0:
                self.servo_actual_pos[i] = min(self.servo_actual_pos[i], self.servo_target_pos[i])
            elif movement_dir < 0:
                self.servo_actual_pos[i] = max(self.servo_actual_pos[i], self.servo_target_pos[i])
                
            pos = int(ServoController.servo_pulse_neutral[i] + ServoController.servo_pulse_range[i] * self.servo_actual_pos[i] / 1000)
            self.pwm.set_pwm(i, 0, pos)
