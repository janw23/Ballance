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
def sqrMagnitude(x, y=None):
    if y is not None:
        return x*x + y*y
    return x[0] * x[0] + x[1] * x[1]

#zwraca dlugosc wektora [x, y]
def magnitude(x, y=None):
    if y is not None:
        return math.sqrt(x*x + y*y)
    return math.sqrt(x[0]*x[0] + x[1]*x[1])

#zwraca roznice kwadratowa miedzy target a value
def errorsquare(target, value):
    size = len(target)
    sum = 0.0
    for i in range(size):
        a = int(target[i]) - value[i]
        sum += a * a
        
    return sum
        