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


'''

import numpy as np
from math import *
import matplotlib.pyplot as plt

pi = 3.1415926535

a = np.array([5,0])

def rotation(beta,vec):
    x = vec[0]*cos(beta)-vec[1]*sin(beta)
    y = vec[0]*sin(beta) + vec[1]*cos(beta)
    ans = np.array([x,y])
    return ans

print(rotation(pi/2,a))

        
def CoefAOA(AOA):
    Cl = np.array([])
    Cd = np.array([])
    for i in range(len(AOA)):
        AOAs = AOA[i]
        if AOAs < 0.34906585 and AOAs >-0.34906585:
            Cls = 0.3*sin(4.5*AOAs)+0.1
        elif AOAs > 0.34906585:
            Cls = 0.3*cos(7*(AOAs-pi/9))+0.1
        Cl = np.append(Cl,Cls)
        Cd = np.append(Cd,abs(0.3 + 8*AOAs**3))
    return (Cl,Cd)

AOA = np.linspace(-0.3,1.7,200)

(LiftCoef,DragCoef) = CoefAOA(AOA)
plt.plot(AOA/np.pi*180,LiftCoef,color = "red",linewidth = 2)
plt.plot(AOA/np.pi*180,DragCoef,color = "blue",linewidth = 2)
plt.grid()
plt.xlabel("Angle of Attacks")
plt.ylabel("Coefficient")
plt.show()