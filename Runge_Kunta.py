import math

import matplotlib.pyplot as plt
import numpy as np
#constant
GM_10M6 = 398602544.6
dt = 1    #unit time second
EARTH_RADIUS = 6371000    #unit meter
pi = 3.141592653589793

mass_satellite = 11000    #kg
time = 40000
direction = 0  ##0 refers to thrust to back   ###  1 refers to thrust forward
Tline = 0
massstart  =865800
a_thrust_x = 0
a_thrust_y = 0
###the information of CZ5
##mass 851800   kg
##payload 14000
#total 865800


##booster：#
# thrust:4*2*1340500
#fuelmass 142800 kg
#mass:156600
#T:173

#core first stage
#thrust2*700000
#fuelmass 165300
#mass 186900
#T:480

#core second stage
#thrust 2*88360
#fuel mass 29100
#mass 36000
#T 660
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

def thrust(Tstart,Toff,force,fuel_mass,dir):
    '''
    dir is from 0 to 2*pi 
    the direction from force/a_thrust and velocity direction  ### clockwise
    input radian
    '''
    global Tline
    global a_thrust_x
    global a_thrust_y
    global direction
    global massstart
    global mass_satellite
    global  v0x
    global v0y
    direction = dir
    alpha = CAc(v0x,v0y) + direction
    if Tline >= Tstart and Tline <= Toff:
        a_thrust_x += force/(mass_satellite+massstart)*math.cos(alpha)
        a_thrust_y += force/(mass_satellite+massstart)*math.sin(alpha)
        massstart -= fuel_mass/(Toff-Tstart)
    if massstart <= 0:
        print('error.Run out of fuel')

def seperation(T,mass):
    global Tline
    global massstart
    if T == Tline:
        massstart -= mass

def CAc(x,y):
    length = (x**2 + y**2)**0.5
    ratio = (x)/(length+0.0000000000000001)
    direction_1 = math.acos(ratio)
    if y >= 0:
        return direction_1
    if y < 0:
        direction_2 = 2*pi - direction_1
        return direction_2
#output

    


##### setup initial condition
r0x = 6671200
r0y = 0
v0x = 0
v0y =10900
#####
##### ouput 
position_x =[]
position_y = []
velocity = []
timel = []
mass_list = []
thrust_a = []
distance_list = []
####mainloop
for i in range(time):
    Tline = i
    result = loop(r0x,r0y,v0x,v0y)
    r0x = result[0]
    r0y = result[1]
    v0x = result[2]
    v0y = result[3]
    v = result[4]
    timel.append(i)
    position_x.append(r0x/1000)
    position_y.append(r0y/1000)
    velocity.append(v)
    distance = math.sqrt(r0x**2+r0y**2)
    mass_list.append(massstart+mass_satellite)
    aaa = (a_thrust_x**2+a_thrust_y**2)**0.5
    a_thrust_y = 0
    a_thrust_x = 0
    distance_list.append(distance/10)
    if distance < EARTH_RADIUS:
        print('collsion the earth.End')
        break


theta = np.arange(0,2 * np.pi,0.001)
x = 6371 * np.cos(theta)
y = 6371 * np.sin(theta)
if len(position_x) == len(position_y):
    plt.plot(position_x,position_y,x,y)
    plt.show()
else:
    print('length is not equal')

plt.plot(timel,velocity,timel,mass_list,timel,thrust_a,timel,distance_list)
plt.legend(labels=['velocity','mass','thrust_a','distance'],loc='best')
plt.show()

