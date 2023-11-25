import matplotlib.pyplot as plt
import numpy as np

'''
This file works as the render to visualize the computing result from the file Calcu.py. 
The file is now able to visulize the 2D simulation by matplotlib. 
Besides, 3D rendering will be developed by applying OpenGL.
'''

def extract(X_t,dt):
    p1x = np.array([])
    p1y = np.array([])
    p2x = np.array([])
    p2y = np.array([])
    p1v = np.array([])
    for i in range(int(len(X_t)/8)):
        p1x = np.append(p1x,X_t[i*8])
        p1y = np.append(p1y,X_t[i*8+1])
        p2x = np.append(p2x,X_t[i*8+2])
        p2y = np.append(p2y,X_t[i*8+3])
    Time = np.arange(0,len(X_t)*dt,dt)
    return p1x,p1y,p2x,p2y,p1v,Time

def VisualOrbit(X_t,dt):
    p1x,p1y,p2x,p2y,p1v,Time = extract(X_t,dt)
    plt.plot(p1x,p1y,c = 'red',label = 'Satellite')
    plt.plot(p2x,p2y,c = 'blue',label = 'Moon')
    plt.legend()
    plt.grid()
    plt.gca().set_aspect(1)
    plt.show()

'''
def ParameterTime(X_t,dt,state):
    p1x,p1y,p2x,p2y,p1v,Time = extract(X_t,dt)
    if state == 1:
        plt.plot(Time,p1v)
        plt.show()

'''

