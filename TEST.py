'''
from start import *
a = np.array([1,2])
p = FindperpenVec(a)
print(p)
import matplotlib.pyplot as plt
x = list(range(1, 21))  # epoch array
loss = [2 / (i**2) for i in x]  # loss values array
plt.ion()
plt.show()

def product(x,y):
    return x*y

class calc:
    def __init__(self,a,b):
        self.a = a
        self.b = b
        self.c = 0

    def add(self):
        a = product(self.a,self.b)
        self.c = a
        return a + self.b
    

k = calc(2,3)
print(k.add())
print(k.c)

'''

from math import *

PI = 3.1415926

altitude = 100
angle = 0.2
camber = 9
thickness = 3
wingarea = 150
TempTrop =59 - 0.00356 * altitude
TempStart = -70
pressureTrop = 2116 * ((TempTrop + 459.7) / 518.6)**5.256
pressureStrat = 473.1 * exp(1.73 - 0.000048  * altitude)





