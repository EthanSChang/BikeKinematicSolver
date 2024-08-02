import numpy as np
import scipy
import matplotlib.pyplot as plt
from Link import Link

class FourBar():
    def __init__(self, startPt, links, endPt):
        self.startPt = startPt
        self.links = links #links are defined ccw starting at 1st fixed point and ending at 2nd fixed point
        self.endPt = endPt

        self.wheelLinkNum = None
        self.shockLinkNum = None
        for i in range(len(links)): #find which links wheel and shock are mounted to
            if links[i].wheel:
                self.wheelLinkNum = i

            if links[i].shock:
                self.shockLinkNum = i

    def solve(self, knownLink): #solve for 2 unknown link angles given known link angle
        fixedLink = Link(np.array([self.endPt, self.startPt])) #vector between 2 fixed points

        if knownLink == 1:
            def linkageEqn(angles): #vector loop equation for 2 unknown angles
                return np.array([self.links[0].vector[0].item() + self.links[1].length*np.cos(angles[0]) + self.links[2].length*np.cos(angles[1]) + fixedLink.vector[0].item(),
                                self.links[0].vector[1].item() + self.links[1].length*np.sin(angles[0]) + self.links[2].length*np.sin(angles[1]) + fixedLink.vector[1].item()])
            
            soln = scipy.optimize.fsolve(linkageEqn, (self.links[1].getAngle(), self.links[2].getAngle()))

        elif knownLink == 2:
            def linkageEqn(angles):
                return np.array([self.links[1].vector[0].item() + self.links[0].length*np.cos(angles[0]) + self.links[2].length*np.cos(angles[1]) + fixedLink.vector[0].item(),
                                self.links[1].vector[1].item() + self.links[0].length*np.sin(angles[0]) + self.links[2].length*np.sin(angles[1]) + fixedLink.vector[1].item()])
            
            soln = scipy.optimize.fsolve(linkageEqn, (self.links[0].getAngle(), self.links[2].getAngle()))

        elif knownLink == 3:
            def linkageEqn(angles):
                return np.array([self.links[2].vector[0].item() + self.links[0].length*np.cos(angles[0]) + self.links[1].length*np.cos(angles[1]) + fixedLink.vector[0].item(),
                                self.links[2].vector[1].item() + self.links[0].length*np.sin(angles[0]) + self.links[1].length*np.sin(angles[1]) + fixedLink.vector[1].item()])
            
            soln = scipy.optimize.fsolve(linkageEqn, (self.links[0].getAngle(), self.links[1].getAngle()))
            
        soln = np.insert(soln, knownLink-1, self.links[knownLink-1].getAngle()) #insert known angle into solution array
        self.links[0].setAngle(soln[0]) #update link objects with solution
        self.links[1].setAngle(soln[1])
        self.links[2].setAngle(soln[2])

    def getJointPts(self):
        joint1 = self.startPt + self.links[0].vector.flatten() 
        joint2 = joint1 + self.links[1].vector.flatten()

        return np.array([self.startPt, joint1, joint2, self.endPt])
    
    def getWheelPt(self):
        return (self.getJointPts()[self.wheelLinkNum] + self.links[self.wheelLinkNum].getWheelVector().T).flatten() #joint position + wheel vector = wheel position
    
    def getShockPt(self):
        return (self.getJointPts()[self.shockLinkNum] + self.links[self.shockLinkNum].getShockVector().T).flatten()

    def plot(self):
        joints = self.getJointPts()
        plt.plot(*joints.T)

        if self.wheelLinkNum is not None:
            wheelPts = np.array([joints[self.wheelLinkNum], joints[self.wheelLinkNum] + self.links[self.wheelLinkNum].getWheelVector().flatten(), joints[self.wheelLinkNum + 1]])
            plt.plot(*wheelPts.T)

        if self.shockLinkNum is not None:
            shockPts = np.array([joints[self.shockLinkNum], joints[self.shockLinkNum] + self.links[self.shockLinkNum].getShockVector().flatten(), joints[self.shockLinkNum + 1]])
            plt.plot(*shockPts.T)


    def getInstantCenter(self):
        if self.wheelLinkNum == 0: #if wheel are on 1st or last link, center of rotation is the fixed point of that link
            return self.getJointPts[0]
        elif self.wheelLinkNum == 2:
            return self.getJointPts[3]
        else: #calc instant center as intersection of 2 lines created by links 1 and 3
            joints = self.getJointPts()
            #line of link 1 in form ax + by = c
            a1 = joints[1][1] - joints[0][1] #y2 - y1
            b1 = joints[0][0] - joints[1][0] #x1 - x2
            c1 = joints[0][0] * (joints[1][1] - joints[0][1]) - joints[0][1] * (joints[1][0] - joints[0][0])  #x1 * (y2-y1)- y1 * (x2-x1)

            #line of link 2
            a2 = joints[3][1] - joints[2][1] #y2 - y1
            b2 = joints[2][0] - joints[3][0] #x1 - x2
            c2 = joints[2][0] * (joints[3][1] - joints[2][1]) - joints[2][1] * (joints[3][0] - joints[2][0])  #x1 * (y2-y1)- y1 * (x2-x1)

            det = a1 * b2 - a2 * b1 #determinant of coefficient matrix
            #solve for intersection with cramer's rule
            x = (c1 * b2 - c2 * b1) / det
            y = (a1 * c2 - a2 * c1) / det

            return np.array([x, y])
 