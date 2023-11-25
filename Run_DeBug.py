from start import *
from structure2 import *


p0 = np.array([0,6459000])
v0 = np.array([-9000,0])

'''
T = 14000
Px,Py,Vel,tim = Simu(p0,v0,T)
'''

oce = Spacecraft(p0,v0,0,500,2)
Px,Py,Vel,tim,AO1,ml = oce.Simulation(1800)
#print(Vel)

fig, ax = plt.subplots()
circle = plt.Circle((0, 0), 6371000)
ax.add_artist(circle)
plt.plot(Px,Py,color = '#FF0000')
plt.title('Orbit')
ax.set_aspect('equal')
print(oce.ThrustProfile([(0,0),(10,0),(12,30),(14,0)],[[0,0],[10,0],[12,0.4],[14,0]],12))
'''
plt.plot(tim,AO1,ml)
plt.title('AOA')

'''
plt.show()


