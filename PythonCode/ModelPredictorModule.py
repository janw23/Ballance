import math
#modul przeiwdywania przyszlej pozycji kulki
class ModelPredictor:
    
    modelDelay = 0.2 #czas opoznienia reakcji rzeczywistego urzadzenia
    gravity = 9.81
    anglePerServoSignal = 10.26 * 0.001 * 2 * math.pi / 180 #kat nachylenia plyty przypadajacy na sygnal pozycji serw
    boardSize = 1 / 0.2 #dlugosc boku platformy
    
    def __init__(self):
        self.position = [0.1, 0.1]
        self.velocity = [0.0, 0.0]
        self.servo = [0.0, 0.0]
        
    def GetPosition(self):
        return tuple(self.position)
    
    def Reset(self):
        self.position = [0.1, 0.1]
        self.velocity = [0.0, 0.0]
    
    def SetServos(self, values):
        self.servo[0] = values[0]
        self.servo[1] = -values[1]
        
    def update(self, deltaTime):
        self.velocity[0] += math.sin(self.servo[0] * ModelPredictor.anglePerServoSignal) * ModelPredictor.gravity * deltaTime
        self.velocity[1] += math.sin(self.servo[1] * ModelPredictor.anglePerServoSignal) * ModelPredictor.gravity * deltaTime
        
        self.position[0] += self.velocity[0] * ModelPredictor.boardSize * deltaTime
        self.position[1] += self.velocity[1] * ModelPredictor.boardSize * deltaTime