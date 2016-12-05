import brickpi
import time
import numpy as np


interface = brickpi.Interface()
interface.initialize()

motors=[0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 8.0 #6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 600.0
motorParams.pidParameters.k_i = 700.0 #700.0
motorParams.pidParameters.k_d = 40.0

port = 2
medValue = 0.0
speed = 8.0

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);
interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)

#while loop measuring distance to wall - add to list
#take median value of last 5 -> measured distance
#error = desired distance from wall - measured distance from wall
#if distance approaching some threshold, slow down/stop
  #angle (i.e. velocity) proportional to error


k = 0.1

thresholdDistance =30.0
error = 1.5
usReading =[]

while True:    
    
    #for i in range(0,100):
    result = interface.getSensorValue(port)
    usReading.append(result[0])
    DIST=usReading[0]
    if (len(usReading)>5):
        medValue = np.median([usReading[-5],usReading[-4],usReading[-3],usReading[-2],usReading[-1]])    
        error = medValue - DIST  
        speedadd = k * abs(error)
        if error < 0:
            interface.setMotorRotationSpeedReferences(motors, [speed+speedadd,speed]) 
        elif error > 0:
            interface.setMotorRotationSpeedReferences(motors, [speed,speed+speedadd])
        elif error == 0:
            pass
    print medValue


interface.terminate()
