import numpy as np
import matplotlib.pyplot as plt

class Link():
    def __init__(self, endpoints, wheelPoint = None, shockPoint = None): #end points in order of tail to tip
        self.vector = np.array(np.atleast_2d(endpoints[1]-endpoints[0]).T, float) #initialize vector as difference between endpoints and ensure vector is float
        self.length = np.linalg.norm(self.vector)
        linkAngle = np.arctan2(self.vector[1], self.vector[0]) #angle relative to x-axis

        self.wheel = False
        self.shock = False

        if wheelPoint is not None: #define wheel center relative to link vector
            self.wheel = True
            wheelVector = np.atleast_2d(wheelPoint - endpoints[0]).T #vector from start point of link to wheel point
            wheelVectorAngle = np.arctan2(wheelVector[1], wheelVector[0]) #angle of wheel vector relative to x-axis
            wheelAngle = (wheelVectorAngle - linkAngle).item() # angle to rotate betwween link vector and auxilliary point vector

            self.wheelMatrix = (np.linalg.norm(wheelVector) / self.length) * np.array([[np.cos(wheelAngle), -np.sin(wheelAngle)], [np.sin(wheelAngle), np.cos(wheelAngle)]]) # matrix to multiply link vector with to get auxilliary point vector - rotation + scaling

        if shockPoint is not None: #same for shock point (ie. rocker) as wheel
            self.shock = True
            shockVector = np.atleast_2d(shockPoint - endpoints[0]).T
            shockVectorAngle = np.arctan2(shockVector[1], shockVector[0])
            shockAngle = (shockVectorAngle - linkAngle).item() 

            self.shockMatrix = (np.linalg.norm(shockVector) / self.length) * np.array([[np.cos(shockAngle), -np.sin(shockAngle)], [np.sin(shockAngle), np.cos(shockAngle)]]) 
   
    def rotate(self, dTheta): 
        rotationMatrix = np.array([[np.cos(dTheta), -np.sin(dTheta)], [np.sin(dTheta), np.cos(dTheta)]])
        self.vector = np.matmul(rotationMatrix, self.vector)

    def getAngle(self):
        return np.arctan2(self.vector[1], self.vector[0]).item()
    
    def setAngle(self, angle):
        self.vector[0] = self.length * np.cos(angle)
        self.vector[1] = self.length * np.sin(angle)

    def getWheelVector(self):
        return np.matmul(self.wheelMatrix, self.vector)
    
    def getShockVector(self):
        return np.matmul(self.shockMatrix, self.vector)
        
