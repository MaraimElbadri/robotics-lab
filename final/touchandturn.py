import brickpi
import time
from forwardBackLeftRight import backward, left, right

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
#0 is the left sensor of the robot

touchports = [0,3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
interface.sensorEnable(touchports[0], brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touchports[1], brickpi.SensorType.SENSOR_TOUCH)

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 500#700 or 300
motorParams.pidParameters.k_i = 700#700 or 100
motorParams.pidParameters.k_d = 40

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

forwardAngles = [20, 20]
backwardAngles = [2,2]
biggerBackwardAngles = [7,7]
turningAngles = [3,3]
biggerAngles = [5,5]
speed = 6.0

while True:
    print("begin loop")
    #angle = float(input("Enter a forward angle to rotate (in radians): "))
    
    while True:
        motorAngles = interface.getMotorAngles(motors)
        result_l = interface.getSensorValue(touchports[0])
        result_r = interface.getSensorValue(touchports[1])
        print('res:::',result_l)
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
            

    print "End loop"
    

"""
while True:
    print("begin loop")
    #angle = float(input("Enter a forward angle to rotate (in radians): "))
    interface.increaseMotorAngleReferences(motors,[forwardAngles[0], forwardAngles[1]])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        result_l = interface.getSensorValue(touchports[0])
        result_r = interface.getSensorValue(touchports[1])
        
        if (result_l[0]==1.0 and result_r[0]==1.0):
            print("both sensors")
            backward(motors, biggerBackwardAngles, interface)
            left(motors, biggerAngles, interface)
            print
            
        elif (result_l[0] == 1.0):
            print("left sensor")
            backward(motors, backwardAngles,interface)
            right(motors, turningAngles, interface)
            print
            
        elif (result_r[0] == 1.0):
            print("right sensor")
            backward(motors, backwardAngles, interface)
            left(motors, turningAngles, interface)
            print
            

    print "End loop"
    
"""
interface.terminate()