from start import *


p0 = np.array([0,6429000])
v0 = np.array([-9000,0])

T = 8000
Px,Py,Vel,tim = Simu(p0,v0,T)

fig, ax = plt.subplots()
circle = plt.Circle((0, 0), 6371000)
ax.add_artist(circle)
plt.plot(Px,Py,color = '#FF0000')
plt.title('Orbit')
ax.set_aspect('equal')


plt.show()


