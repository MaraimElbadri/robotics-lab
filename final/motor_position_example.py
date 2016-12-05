import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
touchports = [0, 3]
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
motorParams.pidParameters.k_p = 300
motorParams.pidParameters.k_i = 100
motorParams.pidParameters.k_d = 40

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

def hitBottle():
    result_l = interface.getSensorValue(touchports[0])
    result_r = interface.getSensorValue(touchports[1])
    
    #print ('touch::::::::: ',result_l,result_r)
    
    hit = False
    if (result_l[0]==1.0 and result_r[0]==1.0):
        #print("both sensors")
        hit = True
        print

    elif (result_l[0] == 1.0):
        #print("left sensor")
        hit = True
        print

    elif (result_r[0] == 1.0):
        #print("right sensor")
        hit = True
        print
        
    return hit

angle = 15
while True:
    #angle = float(input("Enter a angle to rotate (in radians): "))
    #interface.startLogging("logtxt.txt")
    bottleCheck = False
    interface.increaseMotorAngleReferences(motors,[angle, angle])
    result_l = interface.getSensorValue(touchports[0])
    result_r = interface.getSensorValue(touchports[1])
    #print "first loop"
    

    while (not interface.motorAngleReferencesReached(motors)) and (result_l[0] == 0.0) and (result_r[0] == 0.0):
        motorAngles = interface.getMotorAngles(motors)
        result_l = interface.getSensorValue(touchports[0])
        result_r = interface.getSensorValue(touchports[1])
        
    print('While loop finished')
    break
        #if motorAngles :
            #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        #time.sleep(0.1)

    #print "Destination reached!"
    

interface.terminate()


