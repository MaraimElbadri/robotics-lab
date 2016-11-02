import time
import sys
import random
import numpy as np

c = 0;
def getRandomX():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomY():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomTheta(): 
    return random.randint(0, 360)

numberOfParticles = 100

line1 = (10, 10, 10, 500) # (x0, y0, x1, y1)
line2 = (20, 20, 500, 200)  # (x0, y0, x1, y1)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)



def navigateToWaypoint(x,y,particles):
    x=float(x)
    y=float(y)
    dist= np.sqrt(x**2+y**2)
    angle = np.arctan2(x/y)+lasttheta
    
    #rotate angle
    
    
    #turn wheels be (14.6/40) * dist
    while True:
    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    particles = [[getRandomX(), getRandomY(), getRandomTheta()] for i in range(numberOfParticles)]
    Currloc=reduce(lambda x,y: (np.array(x)+np.array(y))/float(numberOfParticles), particles)
    
    print "drawParticles:" + str(particles)
    
    c += 1;
    time.sleep(0.25)
    
    
    #calculate angle
    #rotate
    #move distance

while True:
    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    particles = [(getRandomX(), getRandomY(), getRandomTheta()) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    
    c += 1;
    time.sleep(0.25)
