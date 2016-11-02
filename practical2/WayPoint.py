import time
import sys
import random
import numpy as np
import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

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


def navigateToWaypoint():

    points=[[0,0,0]]
    particles = [(0,0,0) for i in range(0,50)]

    while True:
        x=float(input("What x value? "))
        y=float(input("What y value? "))
        print
        a=x
        b=y
        x=x-points[len(points)-1][0]
        y=y-points[len(points)-1][1]
        
        #angle for the new waypoint w.r.t origin
        angleOriginNew = np.arctan(y/x)
        
        if ((angleOriginNew > 0) and (points[len(points)-1][2]>0)) or ( (angleOriginNew <0) and (points[len(points)-1][2] <0) ):
            angleRotate = angleOriginNew-points[len(points)-1][2]
        else:
             angleRotate = angleOriginNew-points[len(points)-1][2]
            
            
        #if points[len(points)-1][2]*a+(points[len(points)-1][1]-points[len(points)-1][0]*points[len(points)-1][2]) > b:
            #angleRotate = angleOriginNew - points[len(points)-1][2]
        #else:
            #angleRotate = angleOriginNew + points[len(points)-1][2]
                #angle to rotate is the angle of new waypoint w.r.t origin + angle of current position w.r.t to origin
        #which is angle of your robot with respect to the origin 
        print('Angle origin New: ')
        print(np.degrees(angleOriginNew))
        print
        print('Previous angle: ')
        print(np.degrees(points[len(points)-1][2]))
        print
        print('Angle to rotate is: ')
        print(np.degrees(angleRotate))
        print


        points.append([x,y,angleRotate])
        print('List of points: ')
        print(points)
        print
        
        #x and y are distance between you current location and the location you would like to go to
        x=points[len(points)-1][0]-points[len(points)-2][0]
        y=points[len(points)-1][1]-points[len(points)-2][1]

        x=float(x)
        print('x: ' + str(x))
        print('y: ' + str(y))
        print
        
        dist = np.sqrt(x**2 + y**2)
        print('Distance to travel: ')
        print(dist)
        print
        
        rads_for_dist = np.sqrt(x**2 + y**2) * (100) * (14.6/float(40))
        print('Rads to rotate for distance is: ')
        print(rads_for_dist)
        print
        

        

        
        rads_for_angle = angleRotate * ( 5.2 / ( np.pi / float(2) ) )
        
        #turn through angle
        interface.increaseMotorAngleReferences(motors,[-rads_for_angle, rads_for_angle])
        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
        print('Turn completed')
        print
        
        #update particles with rotation
        a=np.random.normal(0,0.01,50)
        for i in range(0,50):
            particles[i]=tuple(( particles[i][0], particles[i][1], particles[i][2] + angleRotate + a[i]))
        #print "drawParticles:" + str(particles)

        #drive distance
        interface.increaseMotorAngleReferences(motors,[rads_for_dist, rads_for_dist])
        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
        
        print('Distance completed')
        print
        
        #update distance particles
        c=0
        k = 1
        while c< k * dist:
            e=np.random.normal(0,0.01,50)
            f=np.random.normal(0,0.01,50)
            D=1
            """
            strange that whatever distance we move the error will be the same
            """
            # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
            for i in range(0,50):
                #particles[i]=tuple( ( float(particles[i][0]+(D+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(D+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )
                 particles[i]=tuple( ( float(particles[i][0]+(dist+e[i])*np.cos(particles[i][2])) , float(particles[i][1]+(dist+e[i])*np.sin(particles[i][2]))  , float(particles[i][2]+f[i]) ) )

            #print "drawParticles:" + str(particles)
            c += 1
        time.sleep(0.25)
        
        #calculate particle averages to be saved as current locations
        xavg=0; yavg=0; angleAvg =0;
        for i in range(0,50):
            xavg = xavg+particles[i][0]/float(50)
            yavg = yavg+particles[i][1]/float(50)
            angleAvg = angleAvg+particles[i][2]/float(50)
        points[len(points)-1]= [xavg,yavg,angleAvg]

navigateToWaypoint()    