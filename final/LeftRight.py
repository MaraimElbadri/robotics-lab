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
motorParams1.pidParameters.k_d = 80 #40

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration = 6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = 255/20.0
motorParams2.minPWM = 18.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 700
motorParams2.pidParameters.k_i = 700
motorParams2.pidParameters.k_d = 160 #40

interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)


angle1 =  float(input("Enter a angle to rotate in straight line (in radians): ")) #37
angle2 = float(input("Enter a angle to rotate direction (in radians): ")) #5.3

    
while True:
        
        interface.startLogging("logLeft.txt")
        #Rotate Left
        interface.increaseMotorAngleReferences(motors,[-angle2, angle2])
        print "second loop"


        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
                #if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
            time.sleep(0.5)
        
        interface.stopLogging()
        
        interface.startLogging("logRight.txt")
        #Rotate Right
        interface.increaseMotorAngleReferences(motors,[angle2, -angle2])
        print "second loop"


        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
                #if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
            time.sleep(0.5)          
            
        break
        
