import numpy as np

particles = []
weights = [float(1.0)/len(particles)]*len(particles)
port = 2
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

def calculateLikelihood(float x, float y, float theta, float z):
    #call wall function
    #m = 
    difference = (z-m)**2
    lkhd = np.exp(-difference / (2 * float(0.02) ) ) + 0.05    
    return lkhd
    
def getSonarValue():
    usReading = []
    while len(usReading) < 10:
        result = interface.getSensorValue(port)
        usReading.append(result[0])
    medValue = np.median(usReading)                 
    return medValue
    
    
def distrib():
    n=0
    z = getSonarValue()
    for p in range(len(particles)):
        beta = np.arccos((np.cos(theta)*(Ay - By) + np.sin(theta)*(By-Ax))/((np.sqrt((Ay-By)**2) +         ((Bx-Ax)**2)))
        if (beta >= 40):
            n +=1
        if (n > len(particles)/2):
            break
    if (n < len(particles)/2):
        for p in range(len(particles)):
            lkhd = calculateLikelihood(particles[p][0], particles[p][1], particles[p][2], z)
            weights[p] = weights[p] * lkhd
    return weights
    
def normalisation(weights):
    weights = [weights[p]/sum(weights) for p in range(len(weights))]
    return weights
    
def resample(particles, weights):
    cDist = [0]
    cdist = 0
    newParticles = []
    for w in weights:
        cdist += w
        cDist.append(cDist)
        
    for sample in range(len(particles)):
        randSample = np.random.uniform(0.0,1.0)
        
        #find which particle the rand float corresponds to 
        for i in range(len(cDist)):
            #length of cDist is one greater than the weights as zero added as first element
            #.: take index as i whose max in len(weights)
            if (cDist[i] < randSample) and (randSample <= cDist[i+1]):
                matchingIndex = i
                break
       #add the sampled particle to the new list
       newParticles.append(particles[matchingIndex])
   return newParticles
       
def SLMU():
    
        
            
            
                                            