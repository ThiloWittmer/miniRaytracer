#!/usr/bin python3

from PIL import Image
import numpy as np
from numba import jit, cuda

class Sphere:
    def __init__(self, radius, center):
        self.radius = radius
        self.center = center
    
    def intersectionParameter(self, ray, origin):
        c = self.center
        r = self.radius
        t = np.dot(c-origin, ray) 
        t1= np.square(np.dot(c-origin, ray))
        t1-= (np.dot(origin-c, origin-c) - np.square(r))
        t1 = np.square(t1)
        t-=t1
        if t < 0:
            return None
        #dist = origin + t01*ray
        #dist = np.linalg.norm(t-origin)
        #dist = (dist-1)/dist
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
    def __init__(self, pointsABC, sizesRS):
        self.points = pointsABC
        self.sizes = sizesRS
        
    def intersectionParameter(self, ray, origin):
        a = self.points[0]
        b = self.points[1]
        c = self.points[2]
        r = self.sizes[0]
        s = self.sizes[1]
        
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
# g e n e r a t e a g r a y s c a l e ( one c h a n n e l ) n o i s e image
gs_img1 = np.random.rand(w,h)
gs_img2 = np.random.rand(w,h)
gs_img3 = np.random.rand(w,h)
# s c a l e image v a l u e s t o [ 0 , 2 5 5 ]
gs_img1 = 255* gs_img1
gs_img2 = 255* gs_img2
gs_img3 = 255* gs_img3
# g e n e r a t e 8 B i t PIL image . These a re t r a n s p o s e d compared
# t o numpy a r r a y s .
r = Image.fromarray(gs_img1.astype(np.uint8).T)
g = Image.fromarray(gs_img2.astype(np.uint8).T)
b = Image.fromarray(gs_img3.astype(np.uint8).T)

# make RGB image ( h e r e a l l t h r e e c h a n n e l s a re t h e same )
rgb = [r,g,b]
# merge c h a n n e l s and s a g e image
Image.merge("RGB",rgb).save("noise_image.png")

objectlist = []
sphere1 = Sphere(5, np.array([1,5,-30]))
objectlist.append(sphere1)

sphere2 = Sphere(10, np.array([1,-10,-30]))
objectlist.append(sphere2)

plane1 = Plane(np.array([0,1,0]), np.array([1,4,-30]))
objectlist.append(plane1)

a = np.array([1,0,0])
b = np.array([1,5,0])
c = np.array([3,0,0])

tria1 = Triangle(np.array([a,b,c]), np.array([10,15]))
objectlist.append(tria1)

rayimg = raytrace(w,h, 255, objectlist)
rayimg.save("ersteKugel.png")