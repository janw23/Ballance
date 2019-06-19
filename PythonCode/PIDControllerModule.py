import MathModule as MM

class PIDController:
    
    #operacje zmiany pidow
    def increaseKP(self):
        self.KP += 100
        print("KP = " + str(self.KP))
        
    def increaseKI(self):
        self.KI += 50
        print("KI = " + str(self.KI))
        
    def increaseKD(self):
        self.KD += 100
        print("KD = " + str(self.KD))        
    def decreaseKP(self):
        self.KP -= 100
        print("KP = " + str(self.KP))
        
    def decreaseKI(self):
        self.KI -= 50
        print("KI = " + str(self.KI))
        
    def decreaseKD(self):
        self.KD -= 100
        print("KD = " + str(self.KD))
        
    #ustawia aktualna wartosc
    def setActualValue(self, x, y=None):
        if y is not None:
            self.value_actual[0] = MM.lerp(self.value_actual[0], x, self.value_smoothing)
            self.value_actual[1] = MM.lerp(self.value_actual[1], y, self.value_smoothing)
        else:
            self.value_actual[0] = MM.lerp(self.value_actual[0], x[0], self.value_smoothing)
            self.value_actual[1] = MM.lerp(self.value_actual[1], x[1], self.value_smoothing)
        
    #ustawia docelowa wartosc
    def setTargetValue(self, x, y=None):
        if y is not None:
            self.value_target[0] = x
            self.value_target[1] = y
        else:
            self.value_target[0] = x[0]
            self.value_target[1] = x[1]
    
    def __init__(self):
        self.servo_pos_limit = (1000, 1000)    #ograniczenia wychylen serw (w skali od 0 do 1000)
        self.value_target = [0.5, 0.5]    #docelowa wartosc, ktora ma byc osiagnieta przez kontroler
        self.value_actual = [0.5, 0.5]    #aktualna wartosc
        self.value_smoothing = 1.0       #wspolczynnik wygladzania aktualizacji aktualnej wartosci
        self.value_target_updated = False #czy docelowa wartosc zostala wlasnie zaktualizowana?

        #wspolczynniki kontroli
        self.KP = 1.3 * 1000   #wzmocnienie czesci proporcjonalnej
        self.KI = 0.2 * 1000    #wzmocnienie czesci calkujacej
        self.KD = 2.7 * 1000   #wzmocnienie czesci rozniczkujacej

        #pozycja serwa
        self.x_servo = 0.0
        self.y_servo = 0.0

        #wartosc bledu
        self.x_error = 0.0
        self.y_error = 0.0

        #wartosci poprzednich bledow
        self.x_prev_error = 0.0
        self.y_prev_error = 0.0

        #zmiana bledu w czasie
        self.x_derivative = 0.0
        self.y_derivative = 0.0

        #calkowita suma bledow
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0

    #aktualizuje kontrolea PID
    def update(self, deltaTime):
        #liczenie bledu
        self.x_error = self.value_target[0] - self.value_actual[0]
        self.y_error = self.value_target[1] - self.value_actual[1]

        #liczenie pochodnej
        self.x_derivative = MM.lerp(self.x_derivative, (self.x_error - self.x_prev_error) / deltaTime, 1.0)
        self.y_derivative = MM.lerp(self.y_derivative, (self.y_error - self.y_prev_error) / deltaTime, 1.0)

        #pochodna kwadratowa
        derivative_magnitude = MM.magnitude(self.x_derivative, self.y_derivative)
        self.x_derivative *= derivative_magnitude
        self.y_derivative *= derivative_magnitude

        self.x_prev_error = self.x_error
        self.y_prev_error = self.y_error

        self.x_error_sum += self.x_error * deltaTime #/ (abs(self.x_derivative)+1)
        self.y_error_sum += self.y_error * deltaTime #/ (abs(self.y_derivative)+1)
        
        #base1 square
        error_sum_magnitude = MM.magnitude(self.x_error_sum, self.y_error_sum)+2
        
        #zmiana pozycji serw z uwzglednieniem bledu biezacego, przyszlego oraz przeszlego
        self.x_servo = (self.x_error * self.KP) + (self.x_derivative * self.KD) + (self.x_error_sum * (error_sum_magnitude) * self.KI)
        self.y_servo = (self.y_error * self.KP) + (self.y_derivative * self.KD) + (self.y_error_sum * (error_sum_magnitude) * self.KI)
        
        #print("x = " + str(self.x_error_sum * error_sum_magnitude))
        #print("y = " + str(self.y_error_sum * error_sum_magnitude))
        #print("")
        
        self.x_servo = MM.clamp(self.x_servo, -self.servo_pos_limit[0], self.servo_pos_limit[0])
        self.y_servo = MM.clamp(self.y_servo, -self.servo_pos_limit[1], self.servo_pos_limit[1])
        
        self.x_error_sum = MM.clamp(self.x_error_sum, -1, 1) * 0.995 * 40 * deltaTime
        self.y_error_sum = MM.clamp(self.y_error_sum, -1, 1) * 0.995 * 40 * deltaTime