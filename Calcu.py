import numpy as np

'''
This file defines the basic iteration function in the whole simulation and some constants.
The algorithm is compatible with both 3D and 2D simulation.

Space Vector in this system is consituted by pSat,pMoon,vSat,vMoon, attitude, mass of satellite.

'''

dt = 20 #second
VELOCITY = 3500 #meter per second
Re = 6371000
Me = 5.97237*(10**24)
G = 6.674*10**(-11)


Mm = 7.346e22
Vm = np.array([0,1022])

State = np.array([])

VThrust = 5000
L =10

def deferentialEqua(X,t,func1,func2):
    '''
    func1 : the time based function of thrust
    func2 : the function of joint angle of time
    '''
    v1 = X[2]
    v2 = X[3]
    l1 = np.linalg.norm(X[0])
    l2 = np.linalg.norm(X[1]-X[0])
    l3 = np.linalg.norm(X[1])
    p1 = X[0]-np.array([0,0])
    p2 = X[0]- X[1]
    v3 = -Me*G/(l1**3)*p1-Mm*G/(l2**3)*p2
    v4 = -Me*G/(l3**3)*X[1]
    v5 = func1(t)/VThrust
    v6 = X[6]
    v7 = np.sin(-func2(t))*func1(t)*3/(X[4]*L)
    return np.array([v1,v2,v3,v4,v5,v6,v7])

def Thrust(t):
    return 0

def UniJoiAng(t):
    return 0


def Iteration(X,t,dt):
    X1_ = deferentialEqua(X,t,Thrust,UniJoiAng)

    X2_ = deferentialEqua(X+0.5*dt*X1_,t+0.5*dt,Thrust,UniJoiAng)

    X3_ = deferentialEqua(X+0.5*dt*X2_,t+0.5*dt,Thrust,UniJoiAng)
    X4_ = deferentialEqua(X+dt*X3_,t+dt,Thrust,UniJoiAng)

    X_k = X+dt/6*(X1_+2*X2_+2*X3_+X4_)
    return X_k
    
def Simulation(X,IterationNum,dt):
    X_t = np.array([])
    for i in range(IterationNum):
        X = Iteration(X,dt*(1+i),dt)
        X_t = np.append(X_t,X)
    return X_t,dt

def Gnereration(Magnitude,degree):
    rad = degree*np.pi/180
    x = Magnitude*np.cos(rad)
    y = Magnitude*np.sin(rad)
    return np.array([x,y])



