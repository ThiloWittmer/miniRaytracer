import numpy as np

class Ray:
    
    def __innit__(self, origin, D):
        self.origin = origin
        self.D = np.linalg.norm(D)
        
    def sphereDiscriminant(self, sphere, point=0):
        O = self.origin
        D = self.D
        C = sphere.centre
        r = sphere.radius
        L = np.subtract(C, O)
        
        tca = np.dot(L, D)
        if tca < 0:
            