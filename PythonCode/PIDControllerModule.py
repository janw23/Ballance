class PIDController:
    
    def __init__(self):
        self.ball_target_velocity = [0.0, 0.0]    #aktualna predkosc kulki
        self.ball_actual_velocity = [0.0, 0.0]    #docelowa predkosc kulki

        #wspolczynniki kontroli
        self.KP = 90.0    #wzmocnienie czesci proporcjonalnej
        self.KD = 0.0    #wzmocnienie czesci rozniczkujacej
        self.KI = 40.0    #wzmocnienie czesci calkujacej

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
        self.x_error = self.ball_target_velocity[0] - self.ball_actual_velocity[0]
        self.y_error = self.ball_target_velocity[1] - self.ball_actual_velocity[1]

        self.x_derivative = (self.x_error - self.x_prev_error) / deltaTime
        self.y_derivative = (self.y_error - self.y_prev_error) / deltaTime

        #zmiana pozycji serw z uwzglednieniem uchybu biezacego, przyszlego oraz przeszlego
        self.x_servo += (self.x_error * self.KP) + (self.x_derivative * self.KD) + (self.x_error_sum * self.KI * deltaTime)
        self.y_servo += (self.y_error * self.KP) + (self.y_derivative * self.KD) + (self.y_error_sum * self.KI * deltaTime)

        #self.x_servo = min(self.ServoUpdater.servo_pos_limit[0], max(-self.ServoUpdater.servo_pos_limit[0], self.x_servo))
        #self.y_servo = min(self.ServoUpdater.servo_pos_limit[1], max(-self.ServoUpdater.servo_pos_limit[1], self.y_servo))

        #self.servoUpdater.moveServo(0, -int(self.x_servo))
        #self.servoUpdater.moveServo(1, int(self.y_servo))

        self.x_prev_error = self.x_error
        self.y_prev_error = self.y_error

        self.x_error_sum += self.x_error * deltaTime
        self.y_error_sum += self.y_error * deltaTime

        self.x_error_sum *= 0.95
        self.y_error_sum *= 0.95




