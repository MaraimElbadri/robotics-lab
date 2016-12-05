import time
import sys
import random
import numpy as np
c = 0;
def getRandomX():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomY():
    return random.uniform(0.0,10.0)

def getRandomTheta(): 
    return random.randint(0, 10)

numberOfParticles = 100

line1 = (10, 10, 10, 500) # (x0, y0, x1, y1)
line2 = (10, 10, 500, 10)  # (x0, y0, x1, y1)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
 

D = 10.0
e = 1.0
f = 0.01

particles = [(getRandomX(), getRandomY(), getRandomTheta()) for i in range(numberOfParticles)]

def drawParticles(particles):
    print "drawParticles:"+ str(particles)
    print particles
    
while True:
        # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
        particles = [(x +(D+e)*np.absolute(np.cos(theta)), y +(D+e)*np.absolute(np.sin(theta)), theta+f) for x, y, theta in particles]
        mu = np.average(particles)
        sigma = np.var(particles, axis =0)
        #print "value of mu: " + str(mu[0])
        #print "drawParticles:" + str(particles)
        c += 1;
        f = random.gauss(1,0.5)*f
        e = random.gauss(1, 0.1)*e
        drawParticles(particles)
        print c
        if (c==10):
            
            break
        time.sleep(0.25)

print particles
    
