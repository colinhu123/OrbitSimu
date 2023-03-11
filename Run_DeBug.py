from start import *


p0 = np.array([0,6600000])
v0 = np.array([-10000,0])

T = 7000
print(len(p0))
Px,Py,Vel,tim = Simu(p0,v0,T)
#print(len(Pos[13]))
plt.plot(Px,Py)

plt.show()


