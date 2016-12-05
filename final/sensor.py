import brickpi
from forwardBackLeftRight import backward, forward

touch_port_1 = 2
touch_port_2 = 0

interface = brickpi.Interface()
interface.initialize()

interface.sensorEnable(touch_port_1, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touch_port_2, brickpi.SensorType.SENSOR_TOUCH)
interface.motorEnable(motors[1])
interface.motorEnable(motors[0])

while True:
    result_1 = interface.getSensorValue(touch_port_1)
    result_2 = interface.getSensorValue(touch_port_2)
    if result_1:
        if result_2:
            touched_1 = result_1[0]
            touched_2 = result_2[0]        
            #print '1 %f'% touched_1
            if (touched_1==1.00) or (touched_2==1.00) :     
                print "touched - moving backwards"
                call move backwards
                call turn angle 
                break
    


interface.terminate()
