
from structure2 import *
import matplotlib.pyplot as plt


p0 = np.array([0,6959000])
v0 = np.array([-7000,0])

'''
T = 14000
Px,Py,Vel,tim = Simu(p0,v0,T)
'''

oce = Spacecraft(p0,v0,0,500,20,np.array([-1,0]))

Px,Py,Vel,tim,AO1,ml = oce.Simulation(2500)
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


