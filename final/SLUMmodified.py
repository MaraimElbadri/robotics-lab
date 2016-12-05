import time
import sys
import numpy as np
#import random
import brickpi
import time
import math
from collections import Counter
import sys


interface=brickpi.Interface()
interface.initialize()

port = 2
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams1 = interface.MotorAngleControllerParameters()
motorParams1.maxRotationAcceleration = 6.0
motorParams1.maxRotationSpeed = 12.0
motorParams1.feedForwardGain = 255/20.0
motorParams1.minPWM = 18.0
motorParams1.pidParameters.minOutput = -255
motorParams1.pidParameters.maxOutput = 255
motorParams1.pidParameters.k_p = 700
motorParams1.pidParameters.k_i = 700
motorParams1.pidParameters.k_d = 40 #40

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration = 6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = 255/20.0
motorParams2.minPWM = 18.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 700
motorParams2.pidParameters.k_i = 700
motorParams2.pidParameters.k_d = 40 #40

interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)

#!/usr/bin/env python

# Some suitable functions and data structures for drawing a map and particles


########################################################################################
#DRAWING MAP LAYOUT
###########################################

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random();

def calcTheta():
    return random.randint(0,360);

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 10;
        self.data = [];

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];

    def draw(self):
        canvas.drawParticles(self.data);

canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();

particles = Particles();

#############################################################################
#WALLS DISTANCE FUNCTION

WALL_LIST=[[0,0,0,168],[0,168,84,168],[84,126,84,210],[84,210,168,210],[168,84,168,210],[168,84,210,84],[210,0,210,84],[0,0,210,0]]
#WALL_LIST=[[0,0,0,168],[0,168,84,168],[84,126,84,210],[84,210,168,210],[168,210,168,84],[168,84,210,84],[210,84,210,0],[210,0,0,0]]
eps=0.2

def mdist(Ax,Ay,Bx,By,x,y,theta):
    Num= (By - Ay)*(Ax-x)-(Bx-Ax)*(Ay-y)
    if theta > np.pi:
        theta = theta - 2*np.pi
    Denom=(By-Ay)*np.cos(theta) - (Bx-Ax)*np.sin(theta)
    if(Denom == 0):
        return (-100.0)
    else:
        return(Num/float(Denom))

def walls(x,y,theta):
    x=x*100
    y=y*100
    gl=[]
    for i in range(8):
        m=mdist(WALL_LIST[i][0],WALL_LIST[i][1],WALL_LIST[i][2],WALL_LIST[i][3],x,y,theta)
        #print(i,'m')
        if m > 0:
            #print(m,'m')
            yy=m*np.sin(theta)+y
            xx=m*np.cos(theta)+x
            if (xx <= WALL_LIST[i][2]+eps) and (xx >= WALL_LIST[i][0] -eps ) and (yy <= WALL_LIST[i][3]+eps) and (yy >= WALL_LIST[i][1] - eps):
                gl.append(m)
            #print(gl,'gl')
    #print(gl)
    m = min(gl)
    print('Distance to Wall::::::::::::::::::::::::: ',m)
    return(m/float(100))

#walls(0.1,.1,np.pi/4)

#walls(0,0,np.pi/4)


##################################################################################################################
#LIKELIHOOD ############################
##################################################################################################################

def calculateLikelihood(x, y, theta, z):
    #call wall function
    #print('Find m to wall')
    #print('values for walls function: ' + str(x) + ', ' + str(y) + ', ' + str(theta) + ', ' + str(z))
    m = walls(x,y,theta)
    difference = (z-m)**2
    lkhd = np.exp(-difference / (2 * float(0.02) ) ) + 0.001
    print('x, y, theta, z, m, lkhd: ')
    print(x,y,theta,z,m,lkhd)
    print
    return lkhd

def getSonarValue():
    usReading = []
    while len(usReading) < 10:
        result = interface.getSensorValue(port)
        usReading.append(result[0])
    medValue = np.median(usReading)/float(100)
    print('Sonar Distance ::::::::::::::::::::::::: ',medValue)
    return medValue


def resampleWeights(particles, weights, z):
    n=0
    for p in range(len(particles)):
        lkhd = calculateLikelihood(particles[p][0], particles[p][1], particles[p][2], z)
        weights[p] = weights[p] * lkhd
    weights = [i/float(sum(weights)) for i in weights]
    return(weights)

def resamplePart(particles, weights, z):
    cDist = [0]
    cdist = 0
    newParticles = []
    for w in weights:
        cdist += w
        cDist.append(cdist)
    #print('cuml dist: ' +str(cDist))
    
    for sample in range(len(particles)):
        randSample = np.random.uniform(0.0,1.0)
        #print('rand num' + str(randSample))
        #find which particle the rand float corresponds to 
        for i in range(len(cDist)):
            #length of cDist is one greater than the weights as zero added as first element
            #.: take index as i whose max in len(weights)
            if (cDist[i] < randSample and randSample <= cDist[i+1]):
                #print(randSample <= cDist[i+1])
                matchingIndex = i
                #print('index' + str(matchingIndex))
                break
        #add the sampled particle to the new list
        newParticles.append(particles[matchingIndex])
    return(newParticles)
    #seq=range(50)
    #ind=np.random.choice(seq,size=(1,50),replace=True,p=weights)[0]
    #np.random.uniform
    #ind=ind[0]
    #for i in range(len(particles)):
    #    particles[i] = particles[ind]
    #return(list(newParticles))



#################################################################################
#ROTATE FUNCTION

def rotangle(x,y,a,b,points):
    if len(points) >1:
        lvecx=points[len(points)-1][0] - points[len(points)-2][0]
        lvecy=points[len(points)-1][1] - points[len(points)-2][1]
        cons=points[len(points)-1][1] - (float(lvecy)/float(lvecx))*points[len(points)-1][0]

        v1= np.array([x,y]);
        v1 = v1/float(np.sqrt(sum(v1**2)))
        v2= np.array([lvecx,lvecy]);
        v2 = v2/float(np.sqrt(sum(v2**2)))

        if ( (b >( (float(lvecy)/float(lvecx))*a + cons)) and (lvecx > 0) ) or ( (b < ( (float(lvecy)/float(lvecx))*a + cons)) and (lvecx < 0)):
            angleRotate = math.acos(v1.dot(v2))
        else:
            angleRotate = -math.acos(v1.dot(v2))
    else:
        print('points',points)
        print('tanof',y,'::',x)
        angleRotate = math.atan(y/float(x))
        lvecx=1
        lvecy=0
        v1= np.array([x,y]);
        v1 = v1/float(np.sqrt(sum(v1**2)))
        v2= np.array([lvecx,lvecy]);
        v2 = v2/float(np.sqrt(sum(v2**2)))
    return(angleRotate)

############################################################################################
#TURN ANGLE
def driveangles(L, R):
        interface.increaseMotorAngleReferences(motors,[L, R])
        while not interface.motorAngleReferencesReached(motors) :
              motorAngles = interface.getMotorAngles(motors)
        print('Turn completed')


############################################################################################
#SHOW PARTICLES FUNCTION
margin=0.05*2.10
scale=768/float(0.210+2.10)

def showparticles(particles,margin,scale):
    showparticles = []
    for i in range(0,50):
        showparticles.append( tuple( ( (float(particles[i][0]) + margin)*scale , (float(particles[i][1])+ margin)*scale, float(particles[i][2]) ) ) )
        print('showtuple',tuple( ( (float(particles[i][0]) + margin)*scale , (float(particles[i][1])+ margin)*scale, float(particles[i][2]) ) ) )
    time.sleep(0.05)
    print "drawParticles:" + str(showparticles)
    time.sleep(0.05)

    
###############################################################################################
#MEDIAN FUNCTION
def medians(particles):
    x=[]
    y=[]
    ang=[]
    for i in range(len(particles)):
        x.append(particles[i][0])
        y.append(particles[i][1])
        ang.append(particles[i][2])
    xmed=np.median(x)
    ymed=np.median(y)
    angmed=np.median(ang)
    return([xmed,ymed,angmed])    
############################################################################################
#NAVIGATION
sde=0.1
sda=0.001

def navigateToWaypoint():
    print('start')

    points=[[0.84,0.3,0]]

    particles = [(0.84,0.3,0) for i in range(0,50)]
    weights = [0.02 for i in range(0,50)]
    #targets = [[1.80, 0.3],[1.8,0.54],[1.38,.54],[1.38,1.68],[1.14,1.68],[1.14,0.84],[0.84,0.84],[0.84,0.3]]
    targets = [[1.80, 0.3]]
    for t in targets:
        x= t[0]
        y= t[1]
        a=x #target postion
        b=y #target position
        x=x-points[len(points)-1][0] #target vector
        y=y-points[len(points)-1][1] #target vecttor
        
        print('xval:::::::::::::::::::::::::::',x)
        ################# Angle to Rotate ######################
        angleRotate = rotangle(x,y,a,b,points)

        ######################
        #ROTATION ######################################################################################################
        ######################
        
        rads_for_angle = angleRotate * ( 5.2 / ( np.pi / float(2) ) )
        
        #angle for the new waypoint w.r.t origin


        #x and y are distance between you current location and the location you would like to go to

        #turn through angle
        
        driveangles(-rads_for_angle, rads_for_angle)

        #update particles with rotation and rotation error
        angdev=np.random.normal(0,0.01,50)
        
        for i in range(0,50):
            particles[i]=tuple(( particles[i][0], particles[i][1], particles[i][2] + angleRotate + angdev[i]))

        print(particles)
        showparticles(particles,margin,scale)

        z = getSonarValue()
        z = z/float(100)
        print('sonar value: ' + str(z))

        print('resample particles')
        wsNew=resampleWeights(particles, weights, z)
        particles = resamplePart(particles, wsNew, z)
        weights = [0.02]*50
        print

        time.sleep(3)


        ################################
        #DRIVING FORWARD FOR KSTEPS  ########################################################################################################
        ###############################
        
        dist = np.sqrt(x**2 + y**2)
        rads_for_dist = np.sqrt(x**2 + y**2) * (100) * (14.6/float(40))
        
        
        numDistSteps = int(math.floor(dist/0.2))
        remDist=dist-0.2*numDistSteps
        rads_for_dist_step = (0.2)*(14.6/0.4)
        rads_for_remdist=remDist*(14.6/0.4)
        
        while dist > 0.2:
            
            driveangles(rads_for_dist_step, rads_for_dist_step)
            
            print('Driving Straight: Distance completed')
            
            e=np.random.normal(0,sde,50)
            f=np.random.normal(0,sda,50)
            for i in range(0,50):
                particles[i]=tuple( ( float(particles[i][0]+(0.2+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(0.2+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )

            #print(particles)
            showparticles(particles,margin,scale)
            z = getSonarValue()
            print('Driving Straight: sonar value: ' + str(z))
            time.sleep(1)
        
            wsNew=resampleWeights(particles, weights, z)
            particles = resamplePart(particles, wsNew, z)
            weights = [0.02]*50
            
            showparticles(particles,margin,scale)
            
            x = a-medians(particles)[0]
            y = b-medians(particles)[1]
            rA=rotangle(x,y,a,b,points)
            
            rads_for_angle = rA * ( 5.2 / ( np.pi / float(2) ) )
            driveangles(-rads_for_angle, rads_for_angle)
            
            angdev=np.random.normal(0,0.01,50)
            for i in range(0,50):
                particles[i]=tuple(( particles[i][0], particles[i][1], particles[i][2] + angleRotate + angdev[i]))

            dist = np.sqrt(x**2 + y**2)
            
            
        
        ###################################################################################
        #DRIVING REMAINING DISTANCE
        ###################################################
        
        remDist= dist
        rads_for_remdist=remDist*(14.6/0.4)
        
        driveangles(rads_for_remdist,rads_for_remdist)
        print('Driving Straight: Distance completed')


        #update distance particles

        # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
        e=np.random.normal(0,sde,50)
        f=np.random.normal(0,sda,50)
        for i in range(0,50):
            particles[i]=tuple( ( float(particles[i][0]+(remDist+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(remDist+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )
        
        #print(particles)
        showparticles(particles,margin,scale)

        z = getSonarValue()
        print('Driving Straight: sonar value: ' + str(z))

        print('resample particles')
        wsNew=resampleWeights(particles, weights, z)
        particles = resamplePart(particles, wsNew, z)
        weights = [0.02]*50
        print

        showparticles(particles,margin,scale)

        ############################################################################################
        #ESTIMATE OF CURRENT PARAMETERS
        ####################################

        points.append(medians(particles))
        print('points')
        print(points)
        print

navigateToWaypoint()
