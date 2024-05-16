#!/usr/bin python3

from PIL import Image
import numpy as np


class Sphere:
    def __init__(self, radius, center):
        self.radius = radius
        self.center = center
    
    def intersectionParameter(self, ray, origin):
        c = self.center
        r = self.radius
        t = np.dot(c-origin, ray) 
        t1= np.square(t)
        t1-= (np.dot(origin-c, origin-c) - np.square(r))
        if t1 < 0:
            return None
        t1 = np.sqrt(t1)
        t-=t1
        if t < 0:
            return None

        return t
    
class Plane:
    def __init__(self, vec, centeredP):
        self.n = vec
        self.c = centeredP
        
    def intersectionParameter(self, ray, origin):
        c = self.c
        n = self.n
        t = np.dot(n, (c - origin)) 
        t/= np.dot(n, ray)
        if t < 0:
            return None
        return t
    
class Triangle:
    def __init__(self, pointsABC):
        self.points = pointsABC
        
    def intersectionParameter(self, ray, origin):
        a = self.points[0]
        b = self.points[1]
        c = self.points[2]
        
        u = b - a
        v = c - a
        w = origin - a
        
        trs = 1/np.dot(np.cross(ray,v),u)
        trs *= np.array([np.dot(np.cross(w,u), v), np.dot(np.cross(ray, v), w), np.dot(np.cross(w,u), ray)])
        
        if trs[1] >= 0 and trs[1] <= 1 and trs[2] >= 0 and trs[2] <= 1 and trs[1] + trs[2] <= 1:
            dist = np.linalg.norm(trs-origin)
            return dist
        
        return None

def coordinateSytem(e, c, up, fov, r = (16/9)):
    coordSys = {}
    coordSys['e'] = e
    f = np.subtract(c,e)
    coordSys['f'] = f / np.linalg.norm(f)
    s = np.cross(f, up)
    coordSys['s'] = s / np.linalg.norm(s)
    u = np.cross(s,f)
    coordSys['u'] = u / np.linalg.norm(u)
    coordSys['h'] = 2*(np.tan(fov/2))
    coordSys['w'] = r * coordSys['h']
    
    return coordSys

def primary_ray(x,y,w,h, coordSys):
    p = x/w - 0.5
    p = p * coordSys['s'] + coordSys['f']
    p = p + coordSys['h'] * (y/h - 0.5) * coordSys['u']
    
    p = p / np.linalg.norm(p)
    return p

def raytrace(w, h, background_color, objectlist):
    img = Image.new("L", (w,h))
    coordSys = coordinateSytem(np.array([0,1.8,10]), np.array([0,3,0]), np.array([0,1,0]), np.pi/4)
    for x in range(0,w):
        for y in range(0,h):
            ray = primary_ray(x,y,w,h,coordSys)
            maxdist = float('inf')
            color = background_color
            for object in objectlist:
                hitdist = object.intersectionParameter(ray, coordSys['e'])
                if hitdist and hitdist < maxdist:
                    maxdist = hitdist
                    color = int(maxdist*5)
                    #color = object.color_at(ray)
            img.putpixel((x,y), color)
    return img
    
# image width , h e i g h t
w, h = 600, 400

a = np.array([0,1,1])
b = np.array([-1,3,1])
c = np.array([1,3,1])

objectlist = []
sphere1 = Sphere(0.5, a)
objectlist.append(sphere1)

sphere2 = Sphere(0.5, b)
objectlist.append(sphere2)

sphere3 = Sphere(0.5, c)
objectlist.append(sphere3)

plane1 = Plane(np.array([0,1,0]), np.array([1,4,-30]))
objectlist.append(plane1)


tria1 = Triangle(np.array([a,b,c]))
objectlist.append(tria1)


rayimg = raytrace(w,h, 255, objectlist)
rayimg.save("ersteKugel.png")