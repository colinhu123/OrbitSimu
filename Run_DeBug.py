
from structure2 import *
import matplotlib.pyplot as plt


p0 = np.array([0,9559000])
v0 = np.array([-5400,0])

'''
T = 14000
Px,Py,Vel,tim = Simu(p0,v0,T)
'''

oce = Spacecraft(p0,v0,0,500,20,np.array([-1,0]))

Px,Py,Vel,tim,AO1,ml = oce.Simulation(4500,"Test")
#print(Vel)

fig, ax = plt.subplots()
circle = plt.Circle((0, 0), 6371000)
ax.add_artist(circle)
plt.plot(Px,Py,color = '#FF0000')
plt.title('Orbit')
ax.set_aspect('equal')

'''
plt.plot(tim,AO1,ml)
plt.title('AOA')
'''

plt.show()

plt.plot(tim,AO1)
plt.show()