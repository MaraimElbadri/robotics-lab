import time
import sys
import numpy as np
# import random
import brickpi
import time
import math
from collections import Counter
import sys

interface = brickpi.Interface()
interface.initialize()

port = 2
touchports = [0,3]
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)
interface.sensorEnable(touchports[0], brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touchports[1], brickpi.SensorType.SENSOR_TOUCH)

motors = [0, 1]
head_motor = [2, 3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

interface.motorEnable(head_motor[0])
interface.motorEnable(head_motor[1])

motorParams1 = interface.MotorAngleControllerParameters()
motorParams1.maxRotationAcceleration = 6.0
motorParams1.maxRotationSpeed = 12.0
motorParams1.feedForwardGain = 255 / 20.0
motorParams1.minPWM = 18.0
motorParams1.pidParameters.minOutput = -255
motorParams1.pidParameters.maxOutput = 255
motorParams1.pidParameters.k_p = 400
motorParams1.pidParameters.k_i = 700
motorParams1.pidParameters.k_d = 80  # 40

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration =6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = 255/20.0
motorParams2.minPWM = 18.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 400 # 500
motorParams2.pidParameters.k_i = 700 #5000
motorParams2.pidParameters.K_d = 10 #10

interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams1)
interface.setMotorAngleControllerParameters(head_motor[0],motorParams2)
interface.setMotorAngleControllerParameters(head_motor[1],motorParams2)


# !/usr/bin/env python

# Some suitable functions and data structures for drawing a map and particles


########################################################################################
# DRAWING MAP LAYOUT
###########################################

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80, 3) + 70 * (math.sin(t));  # in cm


def calcY():
    return random.gauss(70, 3) + 60 * (math.sin(2 * t));  # in cm


def calcW():
    return random.random();


def calcTheta():
    return random.randint(0, 360);


# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self, map_size=210):
        self.map_size = map_size;  # in cm;
        self.canvas_size = 768;  # in pixels;
        self.margin = 0.05 * map_size;
        self.scale = self.canvas_size / (map_size + 2 * self.margin);

    def drawLine(self, line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1, y1, x2, y2))

    def drawParticles(self, data):
        display = [(self.__screenX(d[0]), self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self, x):
        return (x + self.margin) * self.scale

    def __screenY(self, y):
        return (self.map_size + self.margin - y) * self.scale


# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self, wall):
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


canvas = Canvas();  # global canvas we are going to draw on

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
mymap.add_wall((0, 0, 0, 168));  # a
mymap.add_wall((0, 168, 84, 168));  # b
mymap.add_wall((84, 126, 84, 210));  # c
mymap.add_wall((84, 210, 168, 210));  # d
mymap.add_wall((168, 210, 168, 84));  # e
mymap.add_wall((168, 84, 210, 84));  # f
mymap.add_wall((210, 84, 210, 0));  # g
mymap.add_wall((210, 0, 0, 0));  # h
mymap.draw();

particles = Particles();

#############################################################################
# WALLS DISTANCE FUNCTION

WALL_LIST = [[0, 0, 0, 168], [0, 168, 84, 168], [84, 126, 84, 210], [84, 210, 168, 210], [168, 84, 168, 210],
             [168, 84, 210, 84], [210, 0, 210, 84], [0, 0, 210, 0]]
# WALL_LIST=[[0,0,0,168],[0,168,84,168],[84,126,84,210],[84,210,168,210],[168,210,168,84],[168,84,210,84],[210,84,210,0],[210,0,0,0]]
eps = 0.3


def mdist(Ax, Ay, Bx, By, x, y, theta):
    Num = (By - Ay) * (Ax - x) - (Bx - Ax) * (Ay - y)
    if theta > np.pi:
        theta = theta - 2 * np.pi
    Denom = (By - Ay) * np.cos(theta) - (Bx - Ax) * np.sin(theta)
    if (Denom == 0):
        return (-100.0)
    else:
        return (Num / float(Denom))


def walls(x, y, theta):
    
    x = x * 100
    y = y * 100
    gl = []
    for i in range(8):
        m = mdist(WALL_LIST[i][0], WALL_LIST[i][1], WALL_LIST[i][2], WALL_LIST[i][3], x, y, theta)
           
        #print(':::::::::::::::M',i,'dist',m)
        if m > 0:
            # print(m,'m')
            yy = m * np.sin(theta) + y
            xx = m * np.cos(theta) + x
            #print('POSITION::::',i,i,i,i,i,i,i,i,i,i,':',medians(particles))
            #print('::::::::::::::: M CORDINATE ',xx,' :: ',yy)
            #print('::::::::::::::: ALLOWED for X',WALL_LIST[i][0] - eps,' :: ', WALL_LIST[i][2] + eps)
            #print('::::::::::::::: ALLOWED for Y',WALL_LIST[i][1] - eps,' :: ', WALL_LIST[i][3] + eps)
            if (xx <= WALL_LIST[i][2] + eps) and (xx >= WALL_LIST[i][0] - eps) and (yy <= WALL_LIST[i][3] + eps) and (
                yy >= WALL_LIST[i][1] - eps):
                gl.append(m)
                # print(gl,'gl')
    #rint(gl)
    m = min(gl)
    #print('Distance to Wall::::::::::::::::::::::::: ', m)
    return (m / float(100))

def headTurn(minm, maxm):
    z_list = []
    sweep = abs(- maxm - minm)
    angle_h = (5.3)/float(36) #10 degree steps
    interface.increaseMotorAngleReferences(head_motor,[0, -maxm]) #max negative to begin facing right
    while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor) #head should now be at 'max' angle
    #print 'starting rotation'
    #test = int(math.floor(2*np.pi/float(angle_h))) 
    for i in range(int(math.floor(sweep/angle_h))):
        #turn head 1 degree
        interface.increaseMotorAngleReferences(head_motor,[0, angle_h])
        #print(interface.motorAngleReferencesReached(head_motor))
        while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor)
        time.sleep(0.1) 
        z = getSonarValue()
        z_list.append(z)
        #z_med = np.median(z_list[-1],z_list[-2],z_list[-3],z_list[-4])
    interface.increaseMotorAngleReferences(head_motor,[0, -minm])
    while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor) #head should now be at zero
    return z_list, angle_h


def findBottleLikelihoods(x, y, theta, minm, maxm):
    '''Returns list of likelihoods that there is a bottle in sensor reading position'''
    z, angleStepSize = headTurn(minm,maxm)
    angleStepSize = (5.3)/float(36) #0.134
    MList = []
    theta = theta - minm
    #print 'theta bottlelikelihood %f'%theta
    for i in range(len(z)):
        m = walls(x, y, theta)
        MList.append(m)
        theta = float(theta + angleStepSize)
        #print m
    #print('SONARVAL::::: ', z)

    #z, MList = throwAway(z, MList)
    #print('Z new:')
    #print(z)

    #print('WALLDIST:::::',MList)

    difference = np.array([0]*len(z))
    difference=np.asarray(z)-np.asarray(MList)
    
    #print("Z-M:::::",difference)
    
    #lkhd = calculateLikelihoodList(z, MList)
    
    return difference, z, angleStepSize
    #return lkhd, z



def throwAway(z, m):
    for i in range(len(z)):
        if (z[i] > 1.2):
            z[i] = -1
            m[i] = -1

    return z, m

def testBottle():
    x , y, angle, lkhds = findBottlePosition(0.84, 0.3, 0, minm, maxm)
    print(lkhds)
    print
    interface.increaseMotorAngleReferences(motors, [-angle, angle])
    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
    print 'turned'
    print('x: ' + str(x) + ' ; y: ' + str(y) + ' ; angle: ' + str(angle))
##################################################################################################################
# LIKELIHOOD ############################
##################################################################################################################
def calculateLikelihoodList(z, m):
    # call wall function
    # print('Find m to wall')
    # print('values for walls function: ' + str(x) + ', ' + str(y) + ', ' + str(theta) + ', ' + str(z))

    lkhd = []
    for i in range(len(m)):
        if (z[i] != -1):
            difference = (z[i] - m[i]) ** 2
            lkhd.append(np.exp(-difference / (2 * float(0.02))) + 0.001)
        else:
            lkhd.append(100)
    return lkhd



def getSonarValue():
    '''
    usReading = []
    while len(usReading) < 10:
        result = interface.getSensorValue(port)
        usReading.append(result[0])
    medValue = np.median(usReading) / float(100)
    #print('Sonar Distance ::::::::::::::::::::::::: ', medValue)
    '''
    usReading = []
    while len(usReading) < 10:
        result = interface.getSensorValue(2)
        usReading.append(result[0])
    medValue = np.median(usReading)/float(100)                
    return medValue

def calculateLikelihood(x, y, theta, z):
    # call wall function
    # print('Find m to wall')
    # print('values for walls function: ' + str(x) + ', ' + str(y) + ', ' + str(theta) + ', ' + str(z))
    m = walls(x, y, theta)
    
    difference = (z - m) ** 2
    lkhd = np.exp(-difference / (2 * float(0.02))) + 0.001
    #print('x, y, theta, z, m, lkhd: ')
    #print(x, y, theta, z, m, lkhd)
    #print
    return lkhd


def resampleWeights(particles, weights, z):
    n = 0
    for p in range(len(particles)):
        lkhd = calculateLikelihood(particles[p][0], particles[p][1], particles[p][2], z)
        weights[p] = weights[p] * lkhd
    weights = [i / float(sum(weights)) for i in weights]
    return (weights)


def resamplePart(particles, weights, z):
    cDist = [0]
    cdist = 0
    newParticles = []
    for w in weights:
        cdist += w
        cDist.append(cdist)
    # print('cuml dist: ' +str(cDist))

    for sample in range(len(particles)):
        randSample = np.random.uniform(0.0, 1.0)
        # print('rand num' + str(randSample))
        # find which particle the rand float corresponds to
        for i in range(len(cDist)):
            # length of cDist is one greater than the weights as zero added as first element
            # .: take index as i whose max in len(weights)
            if (cDist[i] < randSample and randSample <= cDist[i + 1]):
                # print(randSample <= cDist[i+1])
                matchingIndex = i
                # print('index' + str(matchingIndex))
                break
        # add the sampled particle to the new list
        newParticles.append(particles[matchingIndex])
    return (newParticles)
    # seq=range(50)
    # ind=np.random.choice(seq,size=(1,50),replace=True,p=weights)[0]
    # np.random.uniform
    # ind=ind[0]
    # for i in range(len(particles)):
    #    particles[i] = particles[ind]
    # return(list(newParticles))


#################################################################################
# ROTATE FUNCTION

def rotangle(x, y, a, b, points):
    if len(points) > 1:
        lvecx = points[len(points) - 1][0] - points[len(points) - 2][0]
        lvecy = points[len(points) - 1][1] - points[len(points) - 2][1]
        cons = points[len(points) - 1][1] - (float(lvecy) / float(lvecx)) * points[len(points) - 1][0]

        v1 = np.array([x, y]);
        v1 = v1 / float(np.sqrt(sum(v1 ** 2)))
        v2 = np.array([lvecx, lvecy]);
        v2 = v2 / float(np.sqrt(sum(v2 ** 2)))

        if ((b > ((float(lvecy) / float(lvecx)) * a + cons)) and (lvecx > 0)) or (
            (b < ((float(lvecy) / float(lvecx)) * a + cons)) and (lvecx < 0)):
            angleRotate = math.acos(v1.dot(v2))
        else:
            angleRotate = -math.acos(v1.dot(v2))
    else:
        #print('points', points)
        #print('tanof', y, '::', x)
        angleRotate = math.atan(y / float(x))
        lvecx = 1
        lvecy = 0
        v1 = np.array([x, y]);
        v1 = v1 / float(np.sqrt(sum(v1 ** 2)))
        v2 = np.array([lvecx, lvecy]);
        v2 = v2 / float(np.sqrt(sum(v2 ** 2)))
    return (angleRotate)



###############################################################################################
# MEDIAN FUNCTION
def medians(particles):
    x = []
    y = []
    ang = []
    for i in range(len(particles)):
        x.append(particles[i][0])
        y.append(particles[i][1])
        ang.append(particles[i][2])
    xmed = np.median(x)
    ymed = np.median(y)
    angmed = np.median(ang)
    return ([xmed, ymed, angmed])


############################################################################################  

    
def RotateAngle(angleRotate, particles,update):

    rads_for_angle = angleRotate * (5.2 / (np.pi / float(2)))
    
    L= -rads_for_angle
    
    R= rads_for_angle
    
    interface.increaseMotorAngleReferences(motors, [L, R])
    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
    
    angdev = np.random.normal(0, 0.001, 50)
    
    
    for i in range(0, 50):
        particles[i] = tuple((particles[i][0], particles[i][1], particles[i][2] + angleRotate + angdev[i]))
    
    #print('Turn/Drive completed')
    
    if update==1:
        weights = [0.02] * 50
    
        z = getSonarValue()
        z = z / float(100)
        #print('sonar value: ' + str(z))

        #print('resample particles')
        wsNew = resampleWeights(particles, weights, z)
        particles = resamplePart(particles, wsNew, z)
    
    return(particles)


def DriveAngle(dist,particles,update):
    rads_for_dist = dist * (100) * (14.6/float(40))
    L=rads_for_dist
    R=rads_for_dist
    interface.increaseMotorAngleReferences(motors, [L, R])
    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        
    e = np.random.normal(0, 0.001, 50)
    f= np.random.normal(0, 0.001, 50)
    for i in range(50):
        particles[i] = tuple((particles[i][0]+(dist+e[i])*np.cos(particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(particles[i][2]), particles[i][2]))
        
    if update==1:
        weights = [0.02] * 50
    
        z = getSonarValue()
        z = z / float(100)
        #print('sonar value: ' + str(z))

        #print('resample particles')
        wsNew = resampleWeights(particles, weights, z)
        particles = resamplePart(particles, wsNew, z)
    return(particles)

def findBottleLikelihoods(x, y, theta, minm, maxm):
    '''Returns list of likelihoods that there is a bottle in sensor reading position'''
    z, angleStepSize = headTurn(minm,maxm)
    angleStepSize = (5.3)/float(36) #0.134
    MList = []
    theta = theta - minm
    #print 'theta bottlelikelihood %f'%theta
    for i in range(len(z)):
        m = walls(x, y, theta)
        MList.append(m)
        theta = float(theta + angleStepSize)
    
    #print ('WALL DISTANCE::::::::::',MList)
    #print('Z::::: ',z)


    difference = np.array([0]*len(z))
    difference=np.asarray(z)-np.asarray(MList)
    
    #print ("Z-M:::::::::",difference)

    
    #lkhd = calculateLikelihoodList(z, MList)
    
    return difference, z, angleStepSize

def findBottlePosition(x,y,theta,minm, maxm):
    lkhds, z, angleStepSize = findBottleLikelihoods(x,y,theta,minm, maxm)
    #print('ANGLES',angleStepSize)
    #print('SONAR',z)
    #print('DIFF::::',lkhds)
    
    minIndex = np.argmin((lkhds)) #changed
    #print('minIndex: %f'%minIndex)
    diffval= lkhds[minIndex]
    
    dist = 0.15 #z[minIndex]
    angle = theta-minm + minIndex*angleStepSize
    #print 'Angle of bottle position: %f'%angle
    bottleX = dist*np.cos(angle) + x
    bottleY = dist*abs(np.sin(angle)) + y
    return bottleX,bottleY,angle, dist ,diffval   




sde=0.1
sda=0.001

def NAV(particles, target, updates):
    #print('start')
    weights = [0.02 for i in range(0,50)]
    #targets = [[1.80, 0.3],[1.8,0.54],[1.38,.54],[1.38,1.68],[1.14,1.68],[1.14,0.84],[0.84,0.84],[0.84,0.3]]
    targets = [target]
    for t in targets:
        
        cpos=medians(particles)
        x= t[0]
        y= t[1]
        a=x #target postion
        b=y #target position
        x=x-cpos[0] #target vector
        y=y-cpos[1] #target vecttor
        
        #print('xval:::::::::::::::::::::::::::',x)
        ################# Angle to Rotate ######################
        angleRotate = rotangle(x,y,a,b,[[cpos[0],cpos[1]],[cpos[0]+np.cos(cpos[2]),cpos[1]+np.sin(cpos[2])]])

        ######################
        #ROTATION ######################################################################################################
        ######################
        particles = RotateAngle(angleRotate, particles,updates)
        

        ################################
        #DRIVING FORWARD  ########################################################################################################
        ###############################
        
        dist = np.sqrt(x**2 + y**2)
        particles = DriveAngle(dist,particles,updates)
        
        return(particles)

#####################
#CHALLENGE
####################


#GET ANGLE TO TURN FOR BOTTLE 1


points = [[0.84, 0.3, 0]]

particles = [(0.84, 0.3, 0) for i in range(0, 50)]

#DRIVE FORWARD 20cm in DIRECTION

#START LOGGING
#print('PART',particles[0:5])

j=0
while True:
    j=j+1
    meds=medians(particles)
    #print('MEDS::::::',meds)
    x=meds[0]
    y=meds[1]
    bottleX,bottleY,angle, dist, diffval =findBottlePosition(x,y,0,1.134, 1.134)
    
    #print('BOTTLE POS::',bottleX,bottleY,'ANGLE::',angle)
    
    if abs(diffval) < 0.07:
        angle = 0;
    
    particles=RotateAngle(angle,particles,0)
    #print('FIRST THREE PARTICLES', particles[1:3])
    
    startangle = interface.getMotorAngles(motors)[0][0]
    #print('STARTLOG:::',startangle)
    
    touched = 0
    
    t_end = time.time() + 2
    #print('TIME UNTIL::', t_end)

    while time.time() < t_end:
        P1 = interface.getSensorValue(touchports[0])
        P2 = interface.getSensorValue(touchports[1])
        interface.setMotorRotationSpeedReferences(motors,[7-j,7-j])
        if (P1[0] == 1) or (P2[0] == 1):
            touched=1
            interface.setMotorRotationSpeedReferences(motors,[0,0]) #STOP 
            robotangle=(interface.getMotorAngles(motors)[0][0] - startangle)
            dist=(robotangle)*(0.4/float(14.6))
            #print('DIST:::::::::::',dist)
            e = np.random.normal(0, 0.01, 50)
            f= np.random.normal(0, 0.01, 50)
            '''
            for i in range(50):
                particles[i] = tuple((particles[i][0]+(dist+e[i])*np.cos(particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(particles[i][2]), particles[i][2]))
            '''
            #print('PART BEFORE REV::',medians(particles))
            break
    if touched ==1:
        break
    else:
        interface.setMotorRotationSpeedReferences(motors,[0,0]) 
        robotangle=(interface.getMotorAngles(motors)[0][0] - startangle)
        dist=(robotangle)*(0.4/float(14.6))
        for i in range(50):
            particles[i] = tuple((particles[i][0]+dist*np.cos(particles[i][2]), particles[i][1]+dist*np.sin(particles[i][2]), particles[i][2]))
    

#REVERSE BACK

interface.increaseMotorAngleReferences(motors, [-robotangle*0.9, -robotangle*0.9])
while not interface.motorAngleReferencesReached(motors):
    motorAngles = interface.getMotorAngles(motors)
'''
   for i in range(50):
    e = np.random.normal(0, 0.001, 50)
    f= np.random.normal(0, 0.001, 50)
    for i in range(50):
        particles[i] = tuple(( particles[i][0]+(dist+e[i])*np.cos(-particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(-particles[i][2]), particles[i][2]))
'''        
        
#print('PART AFTER REV::',medians(particles))
#print('particlses:::::',particles)
#print('other',(-dist+e[1])*np.cos(particles[1][2]) )
#print('DIST::', dist)
#print('Error',e)
#ROTATE 180 DEGREES
particles = RotateAngle(np.pi/float(2),particles,1)


#DRIVE TO NEXT LOOK POINT

#print('MEDIAN POSITION BEFORE NAVIGATION',medians(particles))

particles=NAV(particles,[1.2,0.7],1)

#print('MEDIAN POSITION AFTER NAVIGATION 1',medians(particles))

particles=NAV(particles,[1.2,1],0)

#print('MEDIAN POSITION AFTER NAVIGATION 2',medians(particles))

#particles = NAV(particles,[1.26, 0.8]) #faceforward


##################################################







#UPDATE PARTICLES/ UPDATE POINTS with x + (speed*(t-t_end))*np.cos(theta) /// y + (speed*(t-t_end))*np.sin(theta)
#######################   
    #SECOND
###################### second

while True:
    meds=medians(particles)
    #print('MEDS::::::',meds)
    print('MEDS',meds)
    x=meds[0]
    y=meds[1]
    bottleX,bottleY,angle, dist, diffval =findBottlePosition(x,y,0,1.13, 1.13)
    
    
    
    #print('BOTTLE POS::',bottleX,bottleY,'ANGLE::',angle)
    
    if abs(diffval) < 0.07:
        angle = 0;
    
    print('ANGLE,DIST,diffval',angle,dist,diffval)
    particles=RotateAngle(angle,particles,0)
    #print('FIRST THREE PARTICLES', particles[1:3])
    
    startangle = interface.getMotorAngles(motors)[0][0]
    #print('STARTLOG:::',startangle)
    
    touched = 0
    
    t_end = time.time() + 3
    #print('TIME UNTIL::', t_end)
    
    while time.time() < t_end:
    # print('iiiiiin loop')
        P1 = interface.getSensorValue(touchports[0])
        P2 = interface.getSensorValue(touchports[1])
        interface.setMotorRotationSpeedReferences(motors,[6,6])
        if (P1[0] == 1) or (P2[0] == 1):
            touched=1
            interface.setMotorRotationSpeedReferences(motors,[0,0]) #STOP 
            robotangle=(interface.getMotorAngles(motors)[0][0] - startangle)
            dist=(robotangle)*(0.4/float(14.6))
            #print('DIST:::::::::::',dist)
            e = np.random.normal(0, 0.01, 50)
            f= np.random.normal(0, 0.01, 50)
            '''
            for i in range(50):
                particles[i] = tuple((particles[i][0]+(dist+e[i])*np.cos(particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(particles[i][2]), particles[i][2]))
            '''
            #print('PART BEFORE REV::',medians(particles))
            break
    if touched ==1:
        break
    else:
        interface.setMotorRotationSpeedReferences(motors,[0,0]) 
        robotangle=(interface.getMotorAngles(motors)[0][0] - startangle)
        dist=(robotangle)*(0.4/float(14.6))
        for i in range(50):
            particles[i] = tuple((particles[i][0]+dist*np.cos(particles[i][2]), particles[i][1]+dist*np.sin(particles[i][2]), particles[i][2]))
    

#REVERSE BACK

interface.increaseMotorAngleReferences(motors, [-robotangle*0.9, -robotangle*0.9])
while not interface.motorAngleReferencesReached(motors):
    motorAngles = interface.getMotorAngles(motors)
'''
   for i in range(50):
    e = np.random.normal(0, 0.001, 50)
    f= np.random.normal(0, 0.001, 50)
    for i in range(50):
        particles[i] = tuple(( particles[i][0]+(dist+e[i])*np.cos(-particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(-particles[i][2]), particles[i][2]))
'''        
        
#print('PART AFTER REV::',medians(particles))
#print('particlses:::::',particles)
#print('other',(-dist+e[1])*np.cos(particles[1][2]) )
#print('DIST::', dist)
#print('Error',e)
#ROTATE 180 DEGREES
particles = RotateAngle(np.pi/float(2),particles,1)


#DRIVE TO NEXT LOOK POINT

#print('MEDIAN POSITION BEFORE NAVIGATION',medians(particles))

particles=NAV(particles,[1.1,1],1)


particles=NAV(particles,[0.5,0.5],1)

particles=NAV(particles,[0.5,0.65],0)
    
    
#UPDATE PARTICLES/ UPDATE POINTS with x + (speed*(t-t_end))*np.cos(theta) /// y + (speed*(t-t_end))*np.sin(theta)
#######################   
    #THIRD
###################### second

while True:
    meds=medians(particles)
    #print('MEDS::::::',meds)
    x=meds[0]
    y=meds[1]
    bottleX,bottleY,angle, dist, diffval =findBottlePosition(x,y,0,1.13, 1.13)
    
    #print('BOTTLE POS::',bottleX,bottleY,'ANGLE::',angle)
    
    if abs(diffval) < 0.07:
        angle = 0;
    
    particles=RotateAngle(angle,particles,0)
    #print('FIRST THREE PARTICLES', particles[1:3])
    
    startangle = interface.getMotorAngles(motors)[0][0]
    #print('STARTLOG:::',startangle)
    
    touched = 0
    
    t_end = time.time() + 3
    #print('TIME UNTIL::', t_end)

    while time.time() < t_end:
        P1 = interface.getSensorValue(touchports[0])
        P2 = interface.getSensorValue(touchports[1])
        interface.setMotorRotationSpeedReferences(motors,[6,6])
        if (P1[0] == 1) or (P2[0] == 1):
            touched=1
            interface.setMotorRotationSpeedReferences(motors,[0,0]) #STOP 
            robotangle=(interface.getMotorAngles(motors)[0][0] - startangle)
            dist=(robotangle)*(0.4/float(14.6))
            #print('DIST:::::::::::',dist)
            e = np.random.normal(0, 0.01, 50)
            f= np.random.normal(0, 0.01, 50)
            '''
            for i in range(50):
                particles[i] = tuple((particles[i][0]+(dist+e[i])*np.cos(particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(particles[i][2]), particles[i][2]))
            '''
            #print('PART BEFORE REV::',medians(particles))
            break
    if touched ==1:
        break
    else:
        interface.setMotorRotationSpeedReferences(motors,[0,0]) 
        robotangle=(interface.getMotorAngles(motors)[0][0] - startangle)
        dist=(robotangle)*(0.4/float(14.6))
        for i in range(50):
            particles[i] = tuple((particles[i][0]+dist*np.cos(particles[i][2]), particles[i][1]+dist*np.sin(particles[i][2]), particles[i][2]))
    

#REVERSE BACK

interface.increaseMotorAngleReferences(motors, [-robotangle*0.9, -robotangle*0.9])
while not interface.motorAngleReferencesReached(motors):
    motorAngles = interface.getMotorAngles(motors)
'''
   for i in range(50):
    e = np.random.normal(0, 0.001, 50)
    f= np.random.normal(0, 0.001, 50)
    for i in range(50):
        particles[i] = tuple(( particles[i][0]+(dist+e[i])*np.cos(-particles[i][2]), particles[i][1]+(dist+f[i])*np.sin(-particles[i][2]), particles[i][2]))
'''        
        
#print('PART AFTER REV::',medians(particles))
#print('particlses:::::',particles)
#print('other',(-dist+e[1])*np.cos(particles[1][2]) )
#print('DIST::', dist)
#print('Error',e)
#ROTATE 180 DEGREES
particles = RotateAngle(np.pi/float(2),particles,1)


#DRIVE TO NEXT LOOK POINT

#print('MEDIAN POSITION BEFORE NAVIGATION',medians(particles))

particles=NAV(particles,[0.83,0.3],1)


    
    

    
#navigateToWaypoint()