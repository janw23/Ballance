import math
import MathModule as MM

#modul przeiwdywania przyszlej pozycji kulki
class ModelPredictor:
    
    eta = 0.00001    #stala 'niedokladnosci' obliczen
    modelDelay = 0.2 #czas opoznienia reakcji rzeczywistego urzadzenia
    gravity = 9.81
    anglePerServoSignal = 7.5 * 0.001 * math.pi / 180 #kat nachylenia plyty przypadajacy na sygnal pozycji serw
    boardSize = 1 / 0.2 #dlugosc boku platformy
    servo_speed = 10000    #'prawdziwa' szybkosc ruchu serw

    friction_static = (0.04 * gravity) ** 2    #wspolczynnik tarcia statycznego
    friction_dynamic = 0.002 * gravity    #wspolczynnik tarcia dynamicznego

    def __init__(self):
        self.position = [0.1, 0.1]
        self.velocity = [0.0, 0.0]

        self.servo = [0.0, 0.0]
        self.servo_target = [0.0, 0.0]    #docelowa pozycja serwa

        #self.signal_delayer = MM.SignalDelay(4, (0.5, 0.5))

    def GetPosition(self):
        return tuple(self.position)  #self.signal_delayer.get()
    
    def Reset(self):
        self.position = [0.5, 0.5]
        self.velocity = [0.0, 0.0]
    
    def SetServos(self, values):
        self.servo_target[0] = values[0]
        self.servo_target[1] = -values[1]

    #aktualizuje pozycje serw
    def updateServos(self, deltaTime):
        for i in range(2): #tylko 2 serwa
            
            movement_dir = MM.sign(self.servo_target[i] - self.servo[i])
            self.servo[i] += ModelPredictor.servo_speed * movement_dir * deltaTime
            
            if movement_dir > 0: self.servo[i] = min(self.servo[i], self.servo_target[i])
            elif movement_dir < 0: self.servo[i] = max(self.servo[i], self.servo_target[i])
                
            self.servo[i] = round(self.servo[i])

    def update(self, deltaTime):
        ModelPredictor.updateServos(self, deltaTime)

        accel_x = math.sin(self.servo[0] * ModelPredictor.anglePerServoSignal) * ModelPredictor.gravity
        accel_y = math.sin(self.servo[1] * ModelPredictor.anglePerServoSignal) * ModelPredictor.gravity

        #jesli kulka jest nieruchoma
        if MM.sqrMagnitude(self.velocity) <= ModelPredictor.eta:
            #jesli sila dzialajaca na kulke jest za mala, zeby pokonac sile tarcia statycznego
            if MM.sqrMagnitude(accel_x, accel_y) <= ModelPredictor.friction_static:
                accel_x = 0
                accel_y = 0

        #jesli kulka jest ruchoma
        else:
            #aplikowanie tarcia dynamicznego
            direction = MM.normalized(self.velocity)
            accel_x -= direction[0] * ModelPredictor.friction_dynamic
            accel_y -= direction[1] * ModelPredictor.friction_dynamic

        self.velocity[0] += accel_x * deltaTime
        self.velocity[1] += accel_y * deltaTime
        
        self.position[0] += self.velocity[0] * ModelPredictor.boardSize * deltaTime
        self.position[1] += self.velocity[1] * ModelPredictor.boardSize * deltaTime

        #self.signal_delayer.push(tuple(self.position))
        #self.signal_delayer.tick()