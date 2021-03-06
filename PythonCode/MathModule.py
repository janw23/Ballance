#PRZYDATNE FUNKCJE MATEMATYCZNE
import math
import heapq

#zwrca znak liczby
def sign(num):
    if num > 0: return 1.0
    if num < 0: return -1.0
    return 0.0

def softsign(num):
    if num < 0: return num / (1 - num)
    return num / (1 + num)

#zwraca kwadrat liczby, ale z jej znakiem
def signedSqr(num):
    if num >= 0: return num * num
    return -num * num

#zwraca kwadrat liczby ze znakiem; f(x)=(x+1)^2-1
def signedBaseOneSqr(num):
    if num >= 0: return (num+1)**2 - 1
    return 1 - (1-num)**2

#interpolacja liniowa
def lerp(a, b, c):
    return c*b + (1-c) * a

#zwraca dlugosc wektora [x, y] do kwadratu
def sqrMagnitude(x, y=None):
    if y is not None: return x*x + y*y
    return x[0] * x[0] + x[1] * x[1]

#zwraca dlugosc wektora [x, y]
def magnitude(x, y=None):
    if y is not None: return math.sqrt(x*x + y*y)
    return math.sqrt(x[0]*x[0] + x[1]*x[1])

#zwraca odleglosc miedzy punktami A i B
def distance(A, B):
    x = A[0] - B[0]
    y = A[1] - B[1]
    return math.sqrt(x*x + y*y)

#zwraca kwadrat odleglosci miedzy punktami A i B
def sqrDistance(A, B):
    x = A[0] - B[0]
    y = A[1] - B[1]
    return x*x + y*y

#zwraca znormalizowany wektor [x, y]
def normalized(x, y=None):
    if y is not None:
        if x == 0 and y == 0: return (0, 0)
        mag = magnitude(x, y)
        return (x/mag, y/mag)
    else:
        if x[0] == 0 and x[1] == 0: return (0, 0)
        mag = magnitude(x)
        return (x[0]/mag, x[1]/mag)

#zwraca roznice kwadratowa miedzy target a value
def errorsquare(target, value):
    size = len(target)
    sum = 0.0
    for i in range(size):
        a = int(target[i]) - value[i]
        sum += a * a
        
    return sum
        
#zwraca wartosc 'num' ograniczana przez <_min, _max>
def clamp(num, _min, _max):
    if num > _max: return _max
    elif num < _min: return _min
    return num

#zwraca wartosc iloczynu skalarnego miedzy wektorami A i B
def dot(A, B):
    return A[0] * B[0] + A[1] * B[1]

#zwraca wartosc iloczynu wektorowego miedzy wektorami A i B
def cross(A, B):
    return A[0] * B[1] - A[1] * B[0]

#zwraca mediane liczb z tablicy 'data'
def Median(data):
    order = sorted(data)
    size = len(order)
    if size % 2 == 0:
        size = size // 2
        return (order[size-1] + order[size]) / 2
    return order[size//2]

#kolejka priorytetowa
class PriorityQueue:
    def __init__(self):
        self.elements = []
        
    #dodaje element do kolejki
    def push(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
        
    #zdejmuje i zwraca element z poczatku kolejki
    def pop(self):
        return heapq.heappop(self.elements)[1]
    
    #czy kolejka jest pusta?
    def empty(self):
        return len(self.elements) == 0
        
class MedianFilter:
    #size to wielkosc kernela filtra
    def __init__(self, size):
        self.data = [0.0] * size
        self.size = size
        self.index = 0
        
    #dodaje element do tablicy danych filtra
    def push(self, num):
        self.data[self.index] = num
        self.index += 1
        if self.index == self.size: self.index = 0
        
    #zwraca przefiltrowana wartosc
    def getValue(self):
        return Median(self.data)

#pozwala na symulowanie opoznienia w pomiarach
class SignalDelay:
    #'delay' oznacza, ile pomiarow jest miedzy aktualna wartoscia, a opozniana
    def __init__(self, delay, dtype):
        self.delay = delay
        self.data = [dtype] * delay
        self.index = 0

    #dodaje element do struktury danych
    def push(self, element):
        self.data[self.index] = element

    #zwraca element opozniony o 'delay'
    def get(self):
        return self.data[self.index]

    #symuluje uplyw czasu o 1 jednostke; musi byc wywolane pomiedzy uzyciem 'push' a 'get'
    def tick(self):
        self.index += 1
        if self.index == self.delay:
            self.index = 0