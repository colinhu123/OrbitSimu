import numpy as np
import matplotlib.pyplot as plt
from math import *
import time

'''
In this file, all reference will be the center of Earth. The goal of this 
project is to visualize the process of reentry into atmosphere for a 
spacecraft without an acceleration greater than 1.2g.
##Here may use convex optimization

Spacecraft's original orbit is a circular orbit with 300 km height above 
the ground.

Lift coefficient should be 0.7 and drag coefficient should be 0.3
Spacecraft's net mass is 500kg with 200kg fuel which can be jetted out
at the speed of 2500m/s
Ignore the attitude of the spacecraft and it can hold high temperature

All simulation are in 3D. Anyway, in 2D at firts >_<

Anyway, this will be a long term project to entertain myself.
'''

#some physical constant
dt = 0.1
Me = 5.97237*(10**24)
Re = 6371000
G = 6.674*10**(-11)

#some features of the spacecraft
S = 1.5

#some constant for programming
DIMENSION = 2

##math part


def CrossProduct(a1,a2):
    i,j,k = 0,0,0
    if len(a1) == DIMENSION and len(a2) == DIMENSION:
        i = a1[1]*a2[3] - a1[3]*a2[1]
        j = a1[2]*a2[0] - a1[0]*a2[2]
        k = a1[0]*a2[1] - a1[1]*a2[0]
    Re = np.array([i,j,k])
    return Re



def air_density(height):
    if height >= 0:
        if height < 11000:
            T = 15.04-0.00649*height
            p = 101.29*((T+273.1)/288.08)**5.256
        elif height<25000:
            T = -56.46
            p = 22.65*exp(1.73-0.000157*height)
        elif height >= 25000:
            T = -131.21 + 0.00299*height
            p = 2.488*((T+273.1)/216.6)**(-11.388)
        density = p/(0.2869*(T + 273.1))
    else:
        print('please input valid height')
    
    return density


def accel(p):
    if len(p) == DIMENSION:
        l1 = np.linalg.norm(p)
        a = -Me*G/(l1**3)*p
        return a
    else:
        print('What are you fucking doing!')

def NextStep(p,v):
    if len(p) == DIMENSION and len(v) == DIMENSION:
        p1_ = v
        v1_ = accel(p)

        p2_ = v+v1_*0.5*dt
        v2_ = accel(p+v1_*0.5*dt)

        p3_ = v+v2_*0.5*dt
        v3_ = accel(p+p2_*0.5*dt)

        p4_ = v + v3_*dt
        v4_ = accel(p+p3_*dt)

        p0 = p +dt*(p1_+2*p2_+2*p3_+p4_)/6
        v0 = v + dt*(v1_+2*v2_+2*v3_+v4_)/6
        return p0,v0
    else:
        print('What the hell you are input!')

'''
def loop(r0x,r0y,v0x,v0y):
    ##
    k1r = [v0x,v0y]
    k1v = acceleration(r0x,r0y)
    ##
    k2r = [v0x+k1v[0]*0.5*dt,v0y+k1v[1]*0.5*dt]
    k2v = acceleration(r0x+k1r[0]*0.5*dt,r0y+k1r[1]*0.5*dt)
    ##
    k3r = [v0x+k2v[0]*0.5*dt,v0y+k2v[1]*0.5*dt]
    k3v = acceleration(r0x + k2r[0] * dt*0.5, r0y + k2r[1]*dt*0.5)
    ##
    k4r = [v0x+k3v[0]*dt,v0y+k3v[1]*dt]
    k4v = acceleration(r0x + k3r[0]*dt,r0y + k3r[1]*dt)
    #####
    rx = r0x + dt/6*(k1r[0] + 2 * k2r[0] + 2  *k3r[0] + k4r[0])
    ry = r0y + dt/6*(k1r[1] + 2 * k2r[1] + 2 * k3r[1] + k4r[1])
    vx = v0x + dt/6*(k1v[0] + 2 * k2v[0] + 2 * k3v[0] + k4v[0])
    vy = v0y + dt/6*(k1v[1] + 2 * k2v[1] + 2 * k3v[1] + k4v[1])

    v = (vx**2+vy**2)**0.5
    return [rx,ry,vx,vy,v]

def acceleration(x,y):
    global a_thrust_x
    global a_thrust_y
    global direction
    global v0x
    global v0y
    x = x/1000
    y = y/1000
    rsquare = x**2+y**2
    distance = rsquare**0.5
    a = GM_10M6/rsquare
    result = [(a)/distance*(-x)-a_thrust_x,(a)/distance*(-y)-a_thrust_y]
    return result
'''
def trans(Pos):
    posx = []
    posy = []
    for i in range(len(Pos)):
        posx.append(Pos[i][0])
        posy.append(Pos[i][1])
    return posx,posy


def Simu(p,v,T):
    Pos = [p]
    Vel = [v]
    time = np.array([0])
    period = int(T/dt)
    for i in range(period):
        p2,v2 = NextStep(Pos[-1],Vel[-1])
        Pos.append(p2)
        Vel.append(v2)
        time = np.append(time,dt*(i+1))
    
    if len(Pos) == len(Vel) and len(Pos) == len(time):
        px,py = trans(Pos)
        return px, py, Vel, time

'''
p0 = np.array([0,6600000])
v0 = np.array([-7900,0])
p,v = NextStep(p0,v0)
print(p)
'''

