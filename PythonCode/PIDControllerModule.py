import MathModule as MM

class PIDController:
    
    def increaseKP(self):
        self.KP += 50
        print("KP = " + str(self.KP))
        
    def increaseKI(self):
        self.KI += 50
        print("KI = " + str(self.KI))
        
    def increaseKD(self):
        self.KD += 50
        print("KD = " + str(self.KD))
        
    def decreaseKP(self):
        self.KP -= 50
        print("KP = " + str(self.KP))
        
    def decreaseKI(self):
        self.KI -= 50
        print("KI = " + str(self.KI))
        
    def decreaseKD(self):
        self.KD -= 50
        print("KD = " + str(self.KD))
        
    #ustawia aktualna wartosc, ktora ma zostac osiagnieta
    def setActualValue(self, x, y=None):
        if y is not None:
            self.value_actual[0] = MM.lerp(self.value_actual[0], x, self.value_smoothing)
            self.value_actual[1] = MM.lerp(self.value_actual[1], y, self.value_smoothing)
        else:
            self.value_actual[0] = MM.lerp(self.value_actual[0], x[0], self.value_smoothing)
            self.value_actual[1] = MM.lerp(self.value_actual[1], x[1], self.value_smoothing)
        
    #ustawia docelowa wartosc
    def setTargetValue(self, x, y):
        self.value_target[0] = x
        self.value_target[1] = y
    
    def __init__(self):
        self.servo_pos_limit = (1000, 1000)    #ograniczenia wychylen serw (w skali od 0 do 1000)
        self.value_target = [0.5, 0.5]    #docelowa wartosc, ktora ma byc osiagnieta przez kontroler
        self.value_actual = [0.5, 0.5]    #aktualna wartosc
        self.value_smoothing = 0.8        #wspolczynnik wygladzania aktualizacji aktualnej wartosci

        #wspolczynniki kontroli
        self.KP = 0.9 * 1000   #wzmocnienie czesci proporcjonalnej
        self.KI = 0.1 * 1000    #wzmocnienie czesci calkujacej
        self.KD = 0.4 * 1000   #wzmocnienie czesci rozniczkujacej

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
        self.x_error = self.value_target[0] - self.value_actual[0]
        self.y_error = self.value_target[1] - self.value_actual[1]

        self.x_derivative = (self.x_error - self.x_prev_error) / deltaTime
        self.y_derivative = (self.y_error - self.y_prev_error) / deltaTime

        self.x_prev_error = self.x_error
        self.y_prev_error = self.y_error

        self.x_error_sum += self.x_error * deltaTime
        self.y_error_sum += self.y_error * deltaTime
        
        #zmiana pozycji serw z uwzglednieniem uchybu biezacego, przyszlego oraz przeszlego
        self.x_servo = (self.x_error * self.KP) + (self.x_derivative * self.KD) + (self.x_error_sum * self.KI)
        self.y_servo = (self.y_error * self.KP) + (self.y_derivative * self.KD) + (self.y_error_sum * self.KI)
        
        self.x_servo = min(self.servo_pos_limit[0], max(-self.servo_pos_limit[0], self.x_servo))
        self.y_servo = min(self.servo_pos_limit[1], max(-self.servo_pos_limit[1], self.y_servo))

        self.x_error_sum = max(min(self.x_error_sum, 1.0), -1.0) * 0.98
        self.y_error_sum = max(min(self.y_error_sum, 1.0), -1.0) * 0.98



