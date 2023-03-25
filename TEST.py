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
'''
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




