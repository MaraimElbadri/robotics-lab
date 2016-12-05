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
touchports = [0,3]
port = 2
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

motors = [0, 1]
head_motor = [2, 3]
minm = 1.134
maxm = 1.134

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
interface.motorEnable(head_motor[0])
interface.motorEnable(head_motor[1])
interface.sensorEnable(touchports[0], brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touchports[1], brickpi.SensorType.SENSOR_TOUCH)


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

eps = 0.5

forwardAngles = [20, 20]
backwardAngles = [2,2]
biggerBackwardAngles = [7,7]
turningAngles = [3,3]
biggerAngles = [5,5]
speed = 6.0
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
mymap.add_wall((0, 0, 0, 168));  # a
mymap.add_wall((0, 168, 84, 168));  # b
mymap.add_wall((84, 126, 84, 210));  # c
mymap.add_wall((84, 210, 168, 210));  # d
mymap.add_wall((168, 210, 168, 84));  # e
mymap.add_wall((168, 84, 210, 84));  # f
mymap.add_wall((210, 84, 210, 0));  # g
mymap.add_wall((210, 0, 0, 0));  # h
mymap.draw();


#############################################################################
# WALLS DISTANCE FUNCTION

WALL_LIST = [[0, 0, 0, 168], [0, 168, 84, 168], [84, 126, 84, 210], [84, 210, 168, 210], [168, 84, 168, 210],
             [168, 84, 210, 84], [210, 0, 210, 84], [0, 0, 210, 0]]
# WALL_LIST=[[0,0,0,168],[0,168,84,168],[84,126,84,210],[84,210,168,210],[168,210,168,84],[168,84,210,84],[210,84,210,0],[210,0,0,0]]


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
    #print 'theta walls %f' %theta
    gl = []
    for i in range(8):
        m = mdist(WALL_LIST[i][0], WALL_LIST[i][1], WALL_LIST[i][2], WALL_LIST[i][3], x, y, theta)
        # print(i,'m')
        if m > 0:
            # print(m,'m')
            yy = m * np.sin(theta) + y
            xx = m * np.cos(theta) + x
            if (xx <= WALL_LIST[i][2] + eps) and (xx >= WALL_LIST[i][0] - eps) and (yy <= WALL_LIST[i][3] + eps) and (
                yy >= WALL_LIST[i][1] - eps):
                gl.append(m)
                # print(gl,'gl')
    #print(gl)
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
        #print i
        #turn head 1 degree
        interface.increaseMotorAngleReferences(head_motor,[0, angle_h])
        #print(interface.motorAngleReferencesReached(head_motor))
        while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor)
        time.sleep(0.5)
        z = getSonarValue()
        z_list.append(z)
        #z_med = np.median(z_list[-1],z_list[-2],z_list[-3],z_list[-4])
    print z_list
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
    print 'theta bottlelikelihood %f'%theta
    for i in range(len(z)):
        m = walls(x, y, theta)
        MList.append(m)
        theta = float(theta + angleStepSize)
        #print m
    print('Z: ')
    print(z)
    print
    #z, MList = throwAway(z, MList)
    #print('Z new:')
    #print(z)
    print
    print('M:')
    print(MList)
    print
    difference = np.array([0]*len(z))
    difference=np.asarray(z)-np.asarray(MList)
    
    print "Z-M"
    print difference
    
    
    #lkhd = calculateLikelihoodList(z, MList)
    
    return difference, z, angleStepSize
    #return lkhd, z

def findBottlePosition(x,y,theta,minm, maxm):
    difference, z, angleStepSize = findBottleLikelihoods(x,y,theta,minm, maxm)
    print 'min difference'
    print min(difference)
    if (min(difference)> -0.1):
        #while (min(difference) > -0.1 for i in difference):
        interface.increaseMotorAngleReferences(motors, [15, 15])
        while not interface.motorAngleReferencesReached(motors):
            motorAngles = interface.getMotorAngles(motors)
            findBottlePosition(x, y, theta, minm, maxm)
    else:
        minIndex = np.argmin(difference)
    #angleLog = []
    #for i in range(len(difference)):
    #    if (difference[i]<=-0.2):
    #        angleLog.append(difference[i])
    #newIndex = math.ceil(len(angleLog)/2.0)
    #print angleLog
    print('minIndex')
    print minIndex
    dist = z[minIndex]
    print 'theta bottle position'
    print theta
    angle = theta-minm + minIndex*angleStepSize
    bottleX = dist*np.cos(angle)
    bottleY = dist*abs(np.sin(angle))
    return bottleX,bottleY,angle, difference

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
    angle=angle*3.0
    interface.increaseMotorAngleReferences(motors, [-angle, angle])
    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
    #This will drive towards the robot but not by the right distance
    #while motors not touched, drive 20cm, re-measure
    while True:
        #interface.increaseMotorAngleReferences(motors, [forwardAngles, forwardAngles])
        #while not interface.motorAngleReferencesReached(motors):
        #    motorAngles = interface.getMotorAngles(motors)
        #bump 
        #motorAngles = interface.getMotorAngles(motors)
        result_l = interface.getSensorValue(touchports[0])
        result_r = interface.getSensorValue(touchports[1])
        
        if (result_l[0]==0.0 and result_r[0]==0.0):
            interface.setMotorRotationSpeedReferences(motors, [speed,speed])
            
        elif (result_l[0]==1.0 and result_r[0]==1.0):
            print("both sensors")
            backward(motors, biggerBackwardAngles, interface)
            left(motors, biggerAngles, interface)
            print
            
            
        elif (result_l[0] == 1.0):
            print("left sensor")
            interface.setMotorRotationSpeedReferences(motors, [0,0])
            backward(motors, backwardAngles,interface)
            right(motors, turningAngles, interface)
            print
            
        elif (result_r[0] == 1.0):
            print("right sensor")
            interface.setMotorRotationSpeedReferences(motors, [0,0])
            backward(motors, backwardAngles, interface)
            left(motors, turningAngles, interface)
            print
    
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
        #print 'result'
        #print result
        #if(result[0] <120):
        usReading.append(result[0])
    medValue = np.median(usReading)/float(100)                
    return medValue


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
        print('points', points)
        print('tanof', y, '::', x)
        angleRotate = math.atan(y / float(x))
        lvecx = 1
        lvecy = 0
        v1 = np.array([x, y]);
        v1 = v1 / float(np.sqrt(sum(v1 ** 2)))
        v2 = np.array([lvecx, lvecy]);
        v2 = v2 / float(np.sqrt(sum(v2 ** 2)))
    return (angleRotate)


############################################################################################
# TURN ANGLE
def driveangles(L, R):
    interface.increaseMotorAngleReferences(motors, [L, R])
    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
    print('Turn completed')



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


testBottle()