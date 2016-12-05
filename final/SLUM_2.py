import time
import sys
import random
import numpy as np
import brickpi
import time
import math
from collections import Counter


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


interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams1)


# Some suitable functions and data structures for drawing a map and particles

#------------------------------------------------------------------------------
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
    medValue = np.median(usReading)                 
    return medValue
    
    
def distrib(particles, weights, z):
    n=0
    '''
    for p in range(len(particles)):
        numerator = (np.cos(theta)*(Ay - By) + np.sin(theta)*(By-Ax))
        denom = np.sqrt((Ay-By)**2 + (Bx-Ax)**2)
        
        beta = np.arccos(numerator/denom)
                         
        print(beta)             
        if (beta > 40):
            n +=1
        
        if (n > len(particles)/2):
            break
            
    if (n < len(particles)/2):
    '''
    for p in range(len(particles)):
        lkhd = calculateLikelihood(particles[p][0], particles[p][1], particles[p][2], z)
        weights[p] = weights[p] * lkhd
    return weights
    
def normalisation(weights):
    weights = [weights[p]/sum(weights) for p in range(len(weights))]
    return weights
    
def resample(particles, weights):
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
    return newParticles

def showParticles(particles):
    margin = 0.105
    scale = 768/float(2.31)
    showparticles = [0]*50
    for i in range(0,50):
        #showparticles[i]=tuple( ( float(particles[i][0])*vd+ xshift , -1*float(particles[i][1]*vd) + yshift , float(particles[i][2]) ) )
        showparticles[i]=tuple( ( (float(particles[i][0]) + margin)*scale , (float(particles[i][1])+ margin)*scale, float(particles[i][2]) ) )
    time.sleep(0.05)
    print "drawParticles:" + str(showparticles)
    time.sleep(0.05)
    return 0

    
#--------------------------------------------------------------------------------------------------------

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
    
    def draw(self):
        canvas.drawParticles(self.data);

canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();

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

def mdist(Ax,Ay,Bx,By,x,y,theta):
    Num= (By - Ay)*(Ax-x)-(Bx-Ax)*(Ay-y)
    Denom=(By-Ay)*np.cos(theta) - (Bx-Ax)*np.sin(theta)
    if(Denom == 0):
        return -100.0
    else:   
        return(Num/float(Denom))
        
def walls(x,y,theta):
    mdist1=mdist(0,0,0,168,x,y,theta) #a
    # print 'mdist1'
    # print mdist1
    mdist2=mdist(0,168,84,168,x,y,theta)#b
    # print 'mdist2'
    # print mdist2
    mdist3=mdist(84,126,84,210,x,y,theta)#c
    # print 'mdist3'
    # print mdist3
    mdist4=mdist(84,210,168,210,x,y,theta)#d
    # print 'mdist4'
    #print mdist4
    mdist5=mdist(168,84,168,210,x,y,theta)#e
    #print 'mdist5'
    #print mdist5
    mdist6=mdist(168,84,210,84,x,y,theta)#f
    #print 'mdist6'
    #print mdist6
    mdist7=mdist(210,0,210,84,x,y,theta)#g
    #print 'mdist7'
    #print mdist7
    mdist8=mdist(210,0,0,0,x,y,theta)#h
    #print 'mdist8'
    #print mdist8
    
    Mlist=[mdist1,mdist2,mdist3,mdist4,mdist5,mdist6,mdist7,mdist8]
    
    NewList=[-100]*8 
    #a
    yy=mdist1*np.sin(theta)+y
    xx=mdist1*np.cos(theta)+x
    
    print x,y, theta

    if xx==0 and (yy >=0 and yy<=168 ):
        NewList.append(mdist1)
        print 'case a'
    else:
        pass
        
    #b   
    yy=mdist2*np.sin(theta)+y
    xx=mdist2*np.cos(theta)+x

    if yy==168 and (xx >= 0 and xx<=84 ):
        NewList.append(mdist2)
        print 'case b'
    else:
        pass  
    
    #c
    yy=mdist3*np.sin(theta)+y
    xx=mdist3*np.cos(theta)+x
    
    if xx==84 and (yy >= 126 and yy<=210 ):
        NewList.append(mdist3)
        print 'case c'
    else:
        pass
    #d   
    yy=mdist4*np.sin(theta)+y
    xx=mdist4*np.cos(theta)+x
    
    if yy==210 and (xx >= 84 and xx<=168 ):
        NewList.append(mdist4)
        print 'case d'
    else:
        pass
    
    #e
    yy=mdist5*np.sin(theta)+y
    xx=mdist5*np.cos(theta)+x
    if xx==168 and (yy >= 84 and yy <= 210 ):
        NewList.append(mdist5)
        print('case e')
    else:
        pass
        
    yy=mdist6*np.sin(theta)+y
    xx=mdist6*np.cos(theta)+x
    if yy==84 and (xx >= 168 and xx <= 210 ):
        NewList.append(mdist6)
        print 'case f'
    else:
        pass
        
    yy=mdist7*np.sin(theta)+y
    xx=mdist7*np.cos(theta)+x
    if yy==84 and (xx >= 168 and xx <= 210 ):
        NewList.append(mdist7)
        print 'case g'
    else:
        pass
    
    yy=mdist8*np.sin(theta)+y
    xx=mdist8*np.cos(theta)+x
    if yy==0 and (xx >= 0 and xx <= 210 ):
        NewList.append(mdist8)
        print 'case h'
    else:
        pass

    print NewList
    
    newNewList = []
    
    for i in range (8):
        if NewList[i] > 0.0:
             newNewList.append(float(NewList[i]))
    print 'new new'
    print newNewList
    
    m = min(newNewList)/ float(100)
    return(m)

#################################################################################

def navigateToWaypoint():
    print('start')
    
    points=[[0.84,0.3,0]]

    particles = [(0.84,0.3,0) for i in range(0,50)]
    ws = [0.02 for i in range(0,50)]
    targets = [[1.80, 0.3],[1.8,0.54],[1.38,.54],[1.38,1.68],[1.14,1.68],[1.14,0.84],[0.84,0.84],[0.84,0.3]]
    for t in targets:
        x= t[0]
        y= t[1]
        print
        a=x
        b=y
        x=x-points[len(points)-1][0]
        y=y-points[len(points)-1][1]
        
        ################# Dilan bullshit######################
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
            angleRotate = math.atan(y/float(x))
            lvecx=1 
            lvecy=0
            v1= np.array([x,y]);
            v1 = v1/float(np.sqrt(sum(v1**2)))
            v2= np.array([lvecx,lvecy]);
            v2 = v2/float(np.sqrt(sum(v2**2))) 

        ######################################################
        
        vd = 1
        #xshift = -150
        xshift = 0
        yshift = 0
        #yshift = 800
        #sd distance 0.015
        sde=0.1
        #sd angle
        sda = 0.0001
        margin = 0.105
        scale = 768/float(2.31)
        
        
        #ROTATION ######################################################################################################
        #angle for the new waypoint w.r.t origin
        
        #x and y are distance between you current location and the location you would like to go to
        
        dist = np.sqrt(x**2 + y**2)
        rads_for_dist = np.sqrt(x**2 + y**2) * (100) * (14.6/float(40))
        rads_for_angle = angleRotate * ( 5.2 / ( np.pi / float(2) ) )
        
        #turn through angle
        interface.increaseMotorAngleReferences(motors,[-rads_for_angle, rads_for_angle])
        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
        print('Turn completed')
        print
        
        
        #update particles with rotation and rotation error
        a=np.random.normal(0,0.01,50)
        
        for i in range(0,50):
            particles[i]=tuple(( particles[i][0], particles[i][1], particles[i][2] + angleRotate + a[i]))
        showParticles(particles)
        
        z = getSonarValue()
        z = z/float(100)
        print('sonar value: ' + str(z))
        
        print('update weights with sensor')
        wsNew = distrib(particles, ws, z)
        print
        
        print('normalise weights')
        wsNew = normalisation(wsNew)
        print
        
        print('resample particles')
        particles = resample(particles, wsNew)
        print
        
        #time.sleep(3)
        #print(particles)
        #time.sleep(3)
        #print "drawParticles:" + str(particles)
        
        numDistSteps = int(math.floor(dist/0.2))
        remDist=dist-0.2*numDistSteps
        rads_for_dist_step = (0.2)*(14.6/0.4)
        rads_for_remdist=remDist*(14.6/0.4)

        print('Dist ',dist)
        print('numDist ',numDistSteps)

        #drive distance ########################################################################################################
        for i in range(numDistSteps):
            
            interface.increaseMotorAngleReferences(motors,[rads_for_dist_step, rads_for_dist_step])
            while not interface.motorAngleReferencesReached(motors) :
                motorAngles = interface.getMotorAngles(motors)
        
            print('Driving Straight: Distance completed')

        
        #update distance particles

            e=np.random.normal(0,sde,50)
            f=np.random.normal(0,sda,50)

        # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
            for i in range(0,50):
                #particles[i]=tuple( ( float(particles[i][0]+(D+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(D+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )
                particles[i]=tuple( ( float(particles[i][0]+(0.2+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(0.2+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )
            showParticles(particles)

            
            z = getSonarValue()
            print('Driving Straight: sonar value: ' + str(z))
        
            print('Driving Straight: update weights with sensor')
            wsNew = distrib(particles, ws, z)
        
            print('normalise weights')
            wsNew = normalisation(wsNew)
            
            print('normalised weights')
            #print(wsNew)
            print()
            
            time.sleep(1)
            
            print('resample particles')
            particles = resample(particles, wsNew)
            print
            showParticles(particles)     

        #Remaining distance              # ######################################################
            
        interface.increaseMotorAngleReferences(motors,[rads_for_remdist, rads_for_remdist])
        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
        
        print('Driving Straight: Distance completed')

        
        #update distance particles

        e=np.random.normal(0,sde,50)
        f=np.random.normal(0,sda,50)

        # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
        for i in range(0,50):
            #particles[i]=tuple( ( float(particles[i][0]+(D+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(D+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )
            particles[i]=tuple( ( float(particles[i][0]+(remDist+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(remDist+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )
        showParticles(particles)    

        
        z = getSonarValue()
        print('Driving Straight: sonar value: ' + str(z))

        print('Driving Straight: update weights with sensor')
        wsNew = distrib(particles, ws, z)
        print

        print('normalise weights')
        wsNew = normalisation(wsNew)
        print

        print('resample particles')
        particles = resample(particles, wsNew)
        print
            #print "drawParticles:" + str(particles)
            

        #calculate particle averages to be saved as current locations
        xavg=0; yavg=0; angleAvg =0;
        

        data = Counter(particles)
        data.most_common()   # Returns all unique items and their counts
        xxx=data.most_common(1)[0][0][0]
        yyy=data.most_common(1)[0][0][1]
        Ang=data.most_common(1)[0][0][2]
        
        points.append([xxx,yyy,Ang])
        print('points')
        print(points)
        print
        
def walls_test():
        x = 0.84
        y = 0.3
        theta = 0.1
        walls(x,y,theta)
        '''    
                for i in range(0,50):
            xavg = xavg+particles[i][0]/float(50)
            yavg = yavg+particles[i][1]/float(50)
            angleAvg = angleAvg+particles[i][2]/float(50)
        #points[len(points)-1]= [xavg,yavg,angleAvg]
        points.append([xavg,yavg,angleAvg])
        
        
        
        print('Directions ',v1,':',v2)
        print('Angle :',angleRotate*(360/(2*np.pi)))
        print('List of points: ')
        showpoints=[]
        for i in range(len(points)):
            showpoints.append([points[i][0],points[i][1],points[i][2]*(360/(2*np.pi))])
        print(showpoints)
        print(dist)
        '''
walls_test()
#navigateToWaypoint()    