import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration = 6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = 255/20.0
motorParams2.minPWM = 18.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 400
motorParams2.pidParameters.k_i = 700
motorParams2.pidParameters.k_d = 100 #40

interface.setMotorAngleControllerParameters(motors[0],motorParams2)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)

def straight(angle):
    interface.increaseMotorAngleReferences(motors,[angle, angle])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        
def turn(turnAngle):
    interface.increaseMotorAngleReferences(motors,[turnAngle, -turnAngle])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
    
while True:
    #angle = float(input("Enter a angle to rotate (in radians): "))
    interface.startLogging("logtxt.txt")
    
    straight(30)
    turn(6)
    straight(5)
    turn(-3)
    straight(30)

    
        
    break

interface.terminate()
