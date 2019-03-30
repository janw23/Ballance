import time
import zmq
import math
from time import sleep
import MathModule as MM
from multiprocessing import Process, Value

#program do komunikacji z symulacja w Unity3D
class SimulationCommunicator:
    
    def __init__(self):
        print("SimulationCommunicator object created")
        
        #wartosci-rezultaty dzialania symulacji
        self.result_x = Value('f', 0.0)
        self.result_y = Value('f', 0.0)
        
        self.servo_x = Value('i', 0)
        self.servo_y = Value('i', 0)
        
        #zmienne wartosci
        self.servo_actual_pos = [0, 0]    #aktualna pozycja serwa
        self.servo_target_pos = [0, 0]    #docelowa pozycja serwa
        
        self.refreshDeltaTime = 1 / 60
        
    def getBallPosition(self):    #zwraca pozycje kulki
        return (self.result_x.value, self.result_y.value)

    def StartProcessing(self):   #uruchamia proces przetwarzajacy obraz
        print("Starting simulation communication")
        
        self.process = Process(target=SimulationCommunicator.DoCommunication, args=(self,))
        self.process.daemon = True
        self.process.start()
        
    def StopProcessing(self):
        print("Killing simulation communicator")
        self.servo_x.value = -999
        
    def moveServos(self, tab):
        self.servo_x.value = tab[0]
        self.servo_y.value = tab[1]
                
    def DoCommunication(self):
        timeRefreshed = time.perf_counter()
        
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")
        
        while True:
            if time.perf_counter() - timeRefreshed >= self.refreshDeltaTime:
                timeRefreshed = time.perf_counter()
                
                message = socket.recv()
                message = message.split()
                
                self.result_x.value = float(message[0]) / 10000.0
                self.result_y.value = float(message[1]) / 10000.0
                    
                servo_x = self.servo_x.value
                socket.send_string(str(servo_x) + " " + str(self.servo_y.value))
                if servo_x == -999:
                    break
                
                sleep(0.01)
