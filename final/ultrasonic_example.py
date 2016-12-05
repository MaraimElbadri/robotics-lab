import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

port = 2 # port which ultrasoic sensor is plugged in to


interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

#while loop measuring distance to wall - add to list
#take median value of last 5 -> measured distance
#error = desired distance from wall - measured distance from wall
#if distance approaching some threshold, slow down/stop
  #angle (i.e. velocity) proportional to error


while True:    
    usReading = interface.getSensorValue(port)

    if usReading :
        print usReading
    else:
        print "Failed US reading"

    time.sleep(0.05)

interface.terminate()
