import time
import sys
import random
import numpy as np




def navigateToWaypoint(x,y):
    x=float(x)
    y=float(y)
    dist= np.sqrt(x**2+y**2)
    angle = np.arctan2(y,x)
        
    c=0
    particles = [(0,0,angle) for i in range(0,50)]
    while c<50:
        c += 1
        e=np.random.normal(0,0.01,50)
        f=np.random.normal(0,0.01,50)
        D=dist
    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
        for i in range(0,50):
            particles[i]=( float(particles[i][0]+(D+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(D+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) )

    #particles = map(lambda x: [ float(x[0]+(D+e)*np.cos(x[2])) , float( x[1]+(D+e)*np.sin(x[2]) ) , float(x[2]+f) ] for i in particles)
        print "drawParticles:" + str(particles)
    
        c += 1;
        time.sleep(0.25)

navigateToWaypoint(20,15)      