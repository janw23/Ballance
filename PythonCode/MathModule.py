#PRZYDATNE FUNKCJE MATEMATYCZNE
import math

#zwrca znak liczby
def sign(num):
    if num > 0:
        return 1.0
    
    elif num < 0:
        return -1.0
    
    return 0.0

#interpolacja liniowa
def lerp(a, b, c):
    return c*b + (1-c) * a

#zwraca dlugosc wektora [x, y] do kwadratu
def sqrMagnitude(x, y):
    return x*x + y*y

#zwraca dlugosc wektora [x, y]
def magnitude(x, y):
    return math.sqrt(x*x + y*y)