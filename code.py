import numpy as np
import matplotlib.pyplot as plt
import bpy

class vec(object):
    def __init__(self, *args):
        if len(args) == 1:
            self.v = np.array(args[0])
            if np.size(self.v) == 2:
                self.v = np.append(self.v, 0)
        elif len(args) == 2:
            self.v = np.array([args[0], args[1], 0])
        elif len(args) == 3:
            self.v = np.array(args)
    def mag(self):
        return np.sqrt(np.sum(self.v**2))
    def unit(self):
        return vec(self.v/self.mag())
    def __add__(self,other):
        return vec(self.v+other.v)
    def __sub__(self,other):
        return vec(self.v-other.v)
    def __mul__(self,rhs):
        if isinstance(rhs, vec):
            return np.sum(self.v*rhs.v)
        if isinstance(rhs, int) or isinstance(rhs, float):
            return vec(self.v*rhs)
    def __rmul__(self, rhs):
        return vec(self.v*rhs)
    def __pow__(self,other):
        return vec(np.cross(self.v, other.v))
    def __str__(self):
        return "["+ str(self.v[0])+", "+ str(self.v[1])+", "+ str(self.v[2])+"]"
    def x(self):
        return self.v[0]
    def y(self):
        return self.v[1]
    def z(self):
        return self.v[2]
    
        

class nbody():  # [position, velocity, mass]
    def __init__(self, *args):
        self.n = len(args)
        position = []
        velocity = []
        mass = []
        for k in range (0,self.n):
            position.append(vec(args[k][0]))
            velocity.append(vec(args[k][1]))
            mass.append(args[k][2])
        self.p = np.array(position)
        # self.pp = np.array(position)
        self.vel = np.array(velocity)
        self.m = np.array(mass)
        self.G = 1# 6.6743e-11
    def F(self, *args):
        if len(args) == 2 and args[0] != args[1]:
            i = args[0]
            j = args[1]
            r = self.p[j]-self.p[i]
            return r*(self.G*self.m[i]*self.m[j]/np.power(r.mag(),3))
        elif len(args) == 2 and args[0] == args[1]:
            return vec(0,0,0)
        elif len(args) == 1:
            i = args[0]
            f = vec(0,0,0)
            for j in range(0, self.n):
                f += self.F(i,j)
            return f
    def a(self, *args):
        if len(args) == 1:  
            i = args[0]
            return (1/self.m[i])*self.F(i)
        elif len(args) == 0:
            A = []
            for i in range(0, self.n):
                A.append(self.a(i))
            Ac = np.array(A)
            return Ac
    def COM(self, k = 0):
        if k != 0:
            R = vec(0,0,0)
            for i in range (0, self.n):
                R += self.m[i]*self.vel[i]
            return (1/np.sum(self.m))*R
        R = vec(0,0,0)
        for i in range (0, self.n):
            R += self.m[i]*self.p[i]
        return (1/np.sum(self.m))*R
    def update(self, t):
        a = self.a()
        self.vel += a*(t/2)
        self.p += t*self.vel
        a = self.a()
        self.vel += a*(t/2)
    def Coord(self, duration, dt, s = 0):
        k = int(duration/dt)
        self.Pos = []
        self.Vel = []
        for a in range(0, k):
            self.update(dt)
            self.Pos.append(self.p.copy())
            self.Vel.append(self.vel.copy())
        if s != 0:
            return self.Vel
        else:
            return self.Pos


# vx3 = 0.7494421910777922289898659
# vy3 = 1.1501789857502275024030202
# vx1 = vx2 = -0.5*vx3
# vy1 = vy2 = -0.5*vy3
# x = 0.28603155458485724

T = 1600
dt = 0.1
# a = nbody([[300.00436,175.691247,0],[0.466203685,0.43236573,0],100], [[500,200,0],[-0.93240737, -0.86473146,0],100], [[202.99564,224.308753,0],[0.466203685,0.43236573,0],100])
# a = nbody([[1e7, 0, 0], [0, 1e3, 0], 1e24], [[-1e7, 0, 0], [0, -1e3, 0], 1e24])
a = nbody([[397.00436,175.691247,0],[0.466203685,0.43236573,0],100], [[300,200,0],[-0.93240737, -0.86473146,0],100], [[202.99564,224.308753,0],[0.466203685,0.43236573,0],100])
p = a.Coord(T, dt)
X1 = []
Y1 = []
X2 = []
Y2 = []
X3 = []
Y3 = []



for i in range(0, int(T/dt)):
    X1.append(p[i][0].x()/100)
    Y1.append(p[i][0].y()/100)
plt.plot(X1,Y1, 'k')


for i in range(0, int(T/dt)):
    X2.append(p[i][1].x()/100)
    Y2.append(p[i][1].y()/100)
plt.plot(X2,Y2)

for i in range(0, int(T/dt)):
    X3.append(p[i][2].x()/100)
    Y3.append(p[i][2].y()/100)
plt.plot(X3,Y3)


o1 = bpy.data.objects['Sphere1']
o2 = bpy.data.objects['Sphere2']
o3 = bpy.data.objects['Sphere3']

frame = 0

for j in range (0, int((T/dt)/70)):
    i = 70*frame
    bpy.context.scene.frame_set(frame)
    o1.location = [X1[i], Y1[i], 0]
    o2.location = [X2[i], Y2[i], 0]
    o3.location = [X3[i], Y3[i], 0]
    
    o1.keyframe_insert(data_path = 'location', index = -1)
    o2.keyframe_insert(data_path = 'location', index = -1)
    o3.keyframe_insert(data_path = 'location', index = -1)
    
    frame += 1
