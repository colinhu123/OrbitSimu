from start import *


p0 = np.array([0,6439000])
v0 = np.array([-8000,0])

T = 7000
print(len(p0))
Px,Py,Vel,tim = Simu(p0,v0,T)
print(Vel)
print(Px)
print(Py)
#print(len(Pos[13]))
plt.plot(Px,Py)

plt.show()


