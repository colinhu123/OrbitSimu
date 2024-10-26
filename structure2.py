import numpy as np
from math import *


dt = 1 #second
OUTLET_VELOCITY = 3500 #meter per second
EARTH_RADIUS = 6371000
EARTH_MASS = 5.97237*(10**24)
G = 6.674*10**(-11)

DIMENSION = 2

class Spacecraft:

    def __init__(self,P,v,AOA,mass,S,dir_vec):
        self.position = P
        self.velocity = v
        self.AOA = AOA
        self.ang_pos = dir_vec/np.linalg.norm(dir_vec)
        self.mass = mass
        self.S = S
        self.time = 0
        self.a = 0
        self.moment = 0
        self.length = 4
        self.angVelo = 0
        self.I = self.mass/12*(self.length)**2


    def trans(self,Pos):
        posx = []
        posy = []
        for i in range(len(Pos)):
            posx.append(Pos[i][0])
            posy.append(Pos[i][1])
        return posx,posy

    def ThrustProfile(self,ThrustList,GamblingList,t):
        '''
        ThrsustList should look like: [[time1,thrust1],[time2,thrust2],[time3,thrust3]]
        GamblingList is the same
        '''
        self.listh = ThrustList
        self.lisga = GamblingList
        for i in range(len(self.listh)-1):
            if t <= self.listh[-1][0]:
                if t >= self.listh[i][0] and t <= self.listh[i+1][0]:
                    a1 = self.listh[i][0]
                    b1 = self.listh[i][1]
                    a2 = self.listh[i+1][0]
                    b2 = self.listh[i+1][1]
                    Thrust = (b2-b1)/(a2-a1)*(t-a1)+b1
                    break
            if t<0:
                print('Fucking asshole!')
            else:
                Thrust = 0
        i = 0
        for i in range(len(self.lisga)-1):
            if t <= self.lisga[-1][0]:
                if t >= self.lisga[i][0] and t <= self.lisga[i+1][0]:
                    a1 = self.lisga[i][0]
                    b1 = self.lisga[i][1]
                    a2 = self.lisga[i+1][0]
                    b2 = self.lisga[i+1][1]
                    Thrustgam = (b2-b1)/(a2-a1)*(t-a1)+b1
                    break
            if t<0:
                print('Fucking asshole!')
            else:
                Thrustgam = 0
        self.mass = self.mass - Thrust/OUTLET_VELOCITY*dt
        Thrust_vec = self.rotation(self.AOA-Thrustgam,self.velocity)/np.linalg.norm(self.velocity)*Thrust
        self.moment = Thrust*0.5*self.length*sin(-Thrustgam)
        return Thrust_vec
    
    def gravityGrad(self):
        MH = float(np.cross(self.ang_pos*0.5*self.length,self.earthGravi(self.ang_pos*0.5*self.length)*self.mass/2))
        MT = float(np.cross(self.ang_pos*0.5*self.length,self.earthGravi(-self.ang_pos*0.5*self.length)*self.mass/2))
        self.moment = MH+MT

    def air_density(self):
        density = 0
        height = np.linalg.norm(self.position) - EARTH_RADIUS
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
            raise ValueError("invalid input")
    
        return density

    def rotation(self,beta,vec):
        '''
        beta is the transition angle
        vector is translated one
        '''
        x = vec[0]*cos(beta)-vec[1]*sin(beta)
        y = vec[0]*sin(beta) + vec[1]*cos(beta)
        ans = np.array([x,y])
        return ans


    def CoefficientAOA(self,state):
        '''
        state 0: Coefficient of lift
        state 1: Coefficient of drag
        '''
        if state == 0:
            print(self.AOA)
            if self.AOA < 0.34906585 and self.AOA > -0.34906585:
                Cl = 0.3*sin(4.5*self.AOA)+0.1
            elif self.AOA > 0.34906585:
                Cl = 0.3*cos(7*(self.AOA-pi/9))+0.1
            return Cl
        if state == 1:
            Cd = abs(0.3 + 8*self.AOA**3)
            return Cd


    def AOAcalc(self):
        sta = 90
        S = self.ang_pos
        v = self.velocity
        AOA = np.arccos(np.dot(S,v)/(np.linalg.norm(S)*np.linalg.norm(v)))
        self.AOA = AOA % (2*np.pi)
        S = np.append(S,0)
        v = np.append(v,0)
        z = np.cross(v,S)[2]
        if z < 0:
            sta = -90
        if z > 0:
            sta  = 90
        return sta

    def aeroForce(self):
        density = self.air_density()
        dir = self.AOAcalc()
        lift = 0.5*density*self.CoefficientAOA(0)*self.S*(np.linalg.norm(self.velocity))**2
        drag = 0.5*density*self.CoefficientAOA(1)*self.S*(np.linalg.norm(self.velocity))**2
        v_unit= self.velocity/np.linalg.norm(self.velocity)
        liftVec = self.rotation(dir,v_unit)*lift
        dragVec = -v_unit*drag
        return (liftVec+dragVec)/self.mass


    def earthGravi(self,offset):
        l1 = np.linalg.norm(self.position+offset)
        a = -EARTH_MASS*G/(l1**3)*(self.position+offset)
        return a

    def accel(self,p,v,t):
        if len(p) == DIMENSION:
            self.gravityGrad()
            a_earth = self.earthGravi(np.array([0,0]))
            a_air = self.aeroForce()

            a = a_earth + a_air 
            return a
        else:
            print('What are you fucking doing!')

    def NextStep(self):
        if len(self.position) == DIMENSION and len(self.velocity) == DIMENSION:
            self.I = self.mass/12*(self.length)**2

            p1_ = self.velocity
            v1_ = self.accel(self.position,self.velocity,self.time)
            dir1_ = self.angVelo
            angVelo1_ = self.moment/self.I

            p2_ = self.velocity+v1_*0.5*dt
            v2_ = self.accel(self.position+p1_*0.5*dt,self.velocity+v1_*0.5*dt,self.time+0.5*dt)
            dir2_ = self.angVelo + dir1_*0.5*dt
            angVelo2_ = self.moment/self.I

            p3_ = self.velocity+v2_*0.5*dt
            v3_ = self.accel(self.position+p2_*0.5*dt,self.velocity+v2_*0.5*dt,self.time+0.5*dt)
            dir3_ = self.angVelo + dir2_*0.5*dt
            angVelo3_ = self.moment/self.I

            p4_ = self.velocity + v3_*dt
            v4_ = self.accel(self.position+p3_*dt,self.velocity+v3_*dt,self.time+dt)
            dir4_ = self.angVelo + dir3_*dt
            angVelo4_ = self.moment/self.I

            p0 = self.position +dt*(p1_+2*p2_+2*p3_+p4_)/6
            v0 = self.velocity + dt*(v1_+2*v2_+2*v3_+v4_)/6

            self.ang_pos = self.ang_pos+dt*(dir1_+2*dir2_+2*dir3_+dir4_)/6
            self.angVelo = self.angVelo+ dt*(angVelo1_+angVelo2_*2+2*angVelo3_+angVelo4_)/6
            self.position = p0
            self.velocity = v0
            
            
            #betta = self.omega*dt + 0.5*alpha*dt**2
            #self.omega = self.omega + dt*alpha
            #self.AOA = self.AOA - betta
            #self.AOA = self.AOA%(2*pi)
        else:
            print('What the hell you are input!')

    def Simulation(self,Tfinal,save_config):
        name = save_config
        Pos = [self.position]
        Vel = [self.velocity]
        AO = [self.AOA]
        ml = [self.mass]
        time = np.array([0])
        period = int(Tfinal/dt)
        for i in range(period):
            time = np.append(time,dt*(i+1))
            self.NextStep()
            Pos.append(self.position)
            Vel.append(self.velocity)
            AO.append(self.AOA)
            ml.append(self.mass)
            if np.linalg.norm(self.position) <= 6371000:
                print('Crashing the ground at:',time[-1])
                break
        
    
        if len(Pos) == len(Vel) and len(Pos) == len(time):
            px,py = self.trans(Pos)
            np.save(save_config+"_x.npy",px)
            np.save(save_config+"_y.npy",py)
            np.save(save_config+"_velo.npy",Vel)
            np.save(save_config+"_time.npy",time)
            np.save(save_config+"_mass.npy",ml)
            return px, py, Vel, time,AO,ml
        
        


'''
    def FindperpenVec(self,p):
        if len(p) == DIMENSION:
            x = p[0]
            y = -p[1]
            Result = np.array([y,x])
            Res = Result/(np.linalg.norm(Result))
            return Res


    def CalAirForce(self,p,v):
        h = np.linalg.norm(p) - EARTH_RADIUS
        rou = self.air_density()
        nor_side = self.FindperpenVec(self.rotation(self.AOA,v))
        back_side = -self.rotation(self.AOA,v)/(np.linalg.norm(v))
        drag = 0.5*rou*self.CoefficientAOA(1)*self.S*(np.linalg.norm(v))**2
        lift = 0.5*rou*self.CoefficientAOA(0)*self.S*(np.linalg.norm(v))**2
        if self.AOA <= 0:
            if np.linalg.norm(self.position+100*nor_side) > np.linalg.norm(self.position-100*nor_side):
                lift_a = lift/self.mass*nor_side
            if np.linalg.norm(self.position+100*nor_side) < np.linalg.norm(self.position-100*nor_side):
                lift_a = -lift/self.mass*nor_side
            if np.linalg.norm(self.position+100*nor_side) == np.linalg.norm(self.position-100*nor_side):
                lift_a = np.array([0,0])
        if self.AOA > 0:
            if np.linalg.norm(self.position+100*nor_side) < np.linalg.norm(self.position-100*nor_side):
                lift_a = lift/self.mass*nor_side
            if np.linalg.norm(self.position+100*nor_side) > np.linalg.norm(self.position-100*nor_side):
                lift_a = -lift/self.mass*nor_side
            if np.linalg.norm(self.position+100*nor_side) == np.linalg.norm(self.position-100*nor_side):
                lift_a = np.array([0,0])
        drag_a = drag/self.mass*back_side
        a = drag_a+lift_a
        return a
'''