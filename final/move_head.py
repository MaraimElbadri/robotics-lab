import numpy as np
import brickpi
import math
import time
import os

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
head_motor = [2, 3]


interface.sensorEnable(2, brickpi.SensorType.SENSOR_ULTRASONIC)

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
interface.motorEnable(head_motor[0])
interface.motorEnable(head_motor[1])

motorParams1 = interface.MotorAngleControllerParameters()
motorParams1.maxRotationAcceleration = 6.0
motorParams1.maxRotationSpeed = 12.0
motorParams1.feedForwardGain = 255/20.0
motorParams1.minPWM = 18.0
motorParams1.pidParameters.minOutput = -255
motorParams1.pidParameters.maxOutput = 255
motorParams1.pidParameters.k_p = 400
motorParams1.pidParameters.k_i = 700 
motorParams1.pidParameters.k_d = 80 #40



motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration =6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = 255/20.0
motorParams2.minPWM = 18.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 400 # 500
motorParams2.pidParameters.k_i = 700 #5000
motorParams2.pidParameters.K_d = 10 #10

interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams1)
interface.setMotorAngleControllerParameters(head_motor[0],motorParams2)
interface.setMotorAngleControllerParameters(head_motor[1],motorParams2)


#angle1 =  float(input("Enter a angle to rotate in straight line (in radians): ")) #37
#angle2 = float(input("Enter a angle to rotate direction (in radians): ")) #5.3
#angle_h = 3.2



def getSonarValue():
    usReading = []
    while len(usReading) < 20:
        result = interface.getSensorValue(2)
        print 'result'
        print result
        #if(result[0] <120):
        usReading.append(result[0])    
    medValue = np.median(usReading)/float(100)                
    return medValue


def headTurn(min, max):
    z_list = []
    sweep = abs(- max - min)
    angle_h = (5.3)/float(36) #10 degree steps
    interface.increaseMotorAngleReferences(head_motor,[0, -max]) #max negative to begin facing right
    while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor) #head should now be at 'max' angle
    print 'starting rotation'
    #test = int(math.floor(2*np.pi/float(angle_h))) 
    for i in range(int(math.floor(sweep/angle_h))):
        print i
        #turn head 1 degree
        interface.increaseMotorAngleReferences(head_motor,[0, angle_h])
        #print(interface.motorAngleReferencesReached(head_motor))
        while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor)
        z = getSonarValue()
        z_list.append(z)
        #z_med = np.median(z_list[-1],z_list[-2],z_list[-3],z_list[-4])
    print z_list
    interface.increaseMotorAngleReferences(head_motor,[0, -min])
    while not interface.motorAngleReferencesReached(head_motor) :
            motorAngles = interface.getMotorAngles(head_motor) #head should now be at zero
    return z_list
            
    
    
    
end = 1.13
start = 1.13
headTurn(end, start)
'''
#----------------------------------------------------------------------------------------------------------------
# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 33):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = float(s)
            print "file closed"
            f.close();
        else:
            print "WARNING: Signature does not exist."
        return ls
        
# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls):
    ls.sig = headTurn()
    

# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    dist = 0
    for i in range(len(ls2.sig)):
        dist = dist + (float(ls1.sig[i]-ls2.sig[i]))**2
    return dist

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = LocationSignature();
    characterize_location(ls_obs);
    
    comparisons = []
    print signatures.size
    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx);
        print ls_read.sig
        dist    = compare_signatures(ls_obs, ls_read)
        comparisons.append(dist)
        print "Comparisons "+str(comparisons)
        
    mlSig = comparisons.index(min(comparisons))
    print mlSig
    
    return(mlSig)
# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files(). 
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

signatures = SignatureContainer(5);
#signatures.delete_loc_files()

#learn_location();
recognize_location();


#-------------------------------------------------------------------------------------------------------------------





        
   
while True:
        
        #forward movement
        interface.startLogging("logtxt.txt")

        interface.increaseMotorAngleReferences(motors,[angle1, angle1])
        print "first loop" 

        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
            #if motorAngles :
            #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]

            time.sleep(0.5)

        #Backward Movement
        interface.startLogging("logtxt.txt")

        interface.increaseMotorAngleReferences(motors,[-angle1, -angle1])
        print "first loop" 

        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
            #if motorAngles :
            #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]

            time.sleep(0.5)
        
        #Rotate Left
        interface.increaseMotorAngleReferences(motors,[-angle2, angle2])
        print "second loop"


        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
                #if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
            time.sleep(0.5)
        
        #Rotate Right
        interface.increaseMotorAngleReferences(motors,[angle2, -angle2])
        print "second loop"


        while not interface.motorAngleReferencesReached(motors) :
            motorAngles = interface.getMotorAngles(motors)
                #if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
            time.sleep(0.5)          
            
        break
 '''       
