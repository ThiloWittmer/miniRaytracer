#!/usr/bin python3

from PIL import Image
import numpy as np
import ray

def coordinateSytem(e, c, up, fov, r = (16/9)):
    coordSys = {}
    f = np.subtract(c,e)
    coordSys['f'] = np.linalg.norm(f)
    s = np.cross(f, up)
    coordSys['s'] = np.linalg.norm(s)
    u = np.cross(s,f)
    coordSys['u'] = np.linalg.norm(u)
    coordSys['h'] = 2*(np.tan(fov/2))
    coordSys['w'] = r * coordSys['h']
    
    return coordSys

def primary_ray(x,y,w,h, coordSys):
    p = x/w - 0.5
    p = p * coordSys['s'] + coordSys['f']
    p = p + coordSys['h'] * (y/h - 0.5) * coordSys(['u'])
    
    p = np.linalg.norm(p)
    return p

def raytrace(w, h, background_color, objectlist):
    img = Image.new("L", [w,h])
    coordSys = coordinateSytem(np.array([0,1.8,10]), np.array([0,3,0]), np.array([0,1,0]), np.pi/4)
    for x in range(0,w):
        for y in range(0,h):
            ray = primary_ray(x,y,w,h,coordSys)
            maxdist = float('inf')
            color = background_color
            for object in objectlist:
                hitdist = object.intersectionParameter(ray)
                if hitdist and hitdist < maxdist:
                    maxdist = hitdist
                    color = object.color_at(ray)
            img.putpixel((x,y), color)
    
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