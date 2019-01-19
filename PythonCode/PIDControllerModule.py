import MathModule as MM

class PIDController:
    
    def increaseKP(self):
        self.KP += 1
        print("KP = " + str(self.KP))
        
    def increaseKI(self):
        self.KI += 1
        print("KI = " + str(self.KI))
        
    def increaseKD(self):
        self.KD += 0.05
        print("KD = " + str(self.KD))
        
    def decreaseKP(self):
        self.KP -= 1
        print("KP = " + str(self.KP))
        
    def decreaseKI(self):
        self.KI -= 1
        print("KI = " + str(self.KI))
        
    def decreaseKD(self):
        self.KD -= 0.05
        print("KD = " + str(self.KD))
        
    #ustawia aktualna predkosc kulki do obliczania reakcji serw
    def setActualVelocity(self, x, y):
        self.ball_velocity_actual[0] = MM.lerp(self.ball_velocity_actual[0], x, self.ball_velocity_smoothing)
        self.ball_velocity_actual[1] = MM.lerp(self.ball_velocity_actual[1], y, self.ball_velocity_smoothing)
        
    #ustawia docelowa predkosc kulki
    def setTargetVelocity(self, x, y):
        self.ball_velocity_target[0] = x
        self.ball_velocity_target[1] = y
    
    def __init__(self):
        self.servo_pos_limit = (1000, 1000)    #ograniczenia wychylen serw (w skali od 0 do 1000)
        self.ball_velocity_target = [0.0, 0.0]    #aktualna predkosc kulki
        self.ball_velocity_actual = [0.0, 0.0]    #docelowa predkosc kulki
        self.ball_velocity_smoothing = 0.85        #wspolczynnik wygladzania aktualizacji predkosci kulki

        #wspolczynniki kontroli
        self.KP = 40.0    #wzmocnienie czesci proporcjonalnej
        self.KD = 1.0   #wzmocnienie czesci rozniczkujacej
        self.KI = 20.0    #wzmocnienie czesci calkujacej

        #pozycja serwa
        self.x_servo = 0.0
        self.y_servo = 0.0

        #wartosc uchybu redulacji
        self.x_error = 0.0
        self.y_error = 0.0

        #wartosci poprzednich uchybow
        self.x_prev_error = 0.0
        self.y_prev_error = 0.0

        #zmiana uchybu w czasie
        self.x_derivative = 0.0
        self.y_derivative = 0.0

        #calkowita suma uchybow
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0

    def update(self, deltaTime):    #aktualizowanie PIDow
        self.x_error = self.ball_velocity_target[0] - self.ball_velocity_actual[0]
        self.y_error = self.ball_velocity_target[1] - self.ball_velocity_actual[1]

        self.x_derivative = (self.x_error - self.x_prev_error) / deltaTime
        self.y_derivative = (self.y_error - self.y_prev_error) / deltaTime

        self.x_prev_error = self.x_error
        self.y_prev_error = self.y_error

        self.x_error_sum += self.x_error * deltaTime
        self.y_error_sum += self.y_error * deltaTime
        
        #zmiana pozycji serw z uwzglednieniem uchybu biezacego, przyszlego oraz przeszlego
        self.x_servo += (self.x_error * self.KP) + (self.x_derivative * self.KD) + (self.x_error_sum * self.KI)
        self.y_servo += (self.y_error * self.KP) + (self.y_derivative * self.KD) + (self.y_error_sum * self.KI)
        
        self.x_servo = min(self.servo_pos_limit[0], max(-self.servo_pos_limit[0], self.x_servo))
        self.y_servo = min(self.servo_pos_limit[1], max(-self.servo_pos_limit[1], self.y_servo))

        self.x_error_sum = max(min(self.x_error_sum, 1.0), -1.0) * 0.98
        self.y_error_sum = max(min(self.y_error_sum, 1.0), -1.0) * 0.98




