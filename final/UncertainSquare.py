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


particles = [(100,100,0) for i in range(0,50)]
for h in range(0,4):
    
    
    for i in range(4):
        c=0
        interface.increaseMotorAngleReferences(motors,[14.6/float(4), 14.6/float(4)])
        
        while c<50:
            e=np.random.normal(0,0.01,50)
            f=np.random.normal(0,0.01,50)
            D=1
            # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
            particlesList = list(particles)
            for i in range(0,50):

                particles[i] = tuple((particles[i][0]+(D+e[i])*np.cos(particles[i][2]) , particles[i][1]+(D+e[i])*np.sin(particles[i][2])  , particles[i][2]+f[i]))

            print "drawParticles:" + str(particles)

            c += 1
        

        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)

        time.sleep(0.25)
        
    print "Straight line completed"
    

    #interface.terminate()
    
    interface.increaseMotorAngleReferences(motors,[5.2, -5.2])
    
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
    
    print('motors rotated')
    
    a=np.random.normal(0,0.01,50)
    
    for i in range(0,50):
        particles[i] = tuple((particles[i][0], particles[i][1] , particles[i][2] + (np.pi/float(2)) + a[i] ))

    print "drawParticles:" + str(particles)
    time.sleep(0.25)
    
    
    
    
    
    
    