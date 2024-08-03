import numpy as np
import scipy
import matplotlib.pyplot as plt

class Linkage():
    def __init__(self, wheelDia, frontWheelPos, cgHeight, bbPos, shock, fourBar = None, link = None, mainPivot = None):
        self.wheelDia = wheelDia
        self.frontWheelPos = frontWheelPos
        self.cgHeight = cgHeight
        self.bbPos = bbPos #all dimensions given in same coordinate frame (ie. bb as origin)
        self.singlePivot = False
        self.shock = shock

        if fourBar is not None: #for linkages with a four bar component (i.e. horst, dw, linkage driven single pivot, etc.)
            self.fourBar = fourBar

        if link is not None: #for single pivots
            self.singlePivot = True
            self.singlePivotLink = link
            self.mainPivot = mainPivot


    def solveLinkage(self, linkNum, angleIncrement): #link num is which link to sweep through angle range
        #angle increment is signed, + for ccw, - for cw

        if not self.singlePivot: #todo: single pivot
            self.fourBar.solve(linkNum) #put initial conditions into array
            self.joint1 = np.array(np.atleast_2d(self.fourBar.getJointPts()[1]))
            self.joint2 = np.array(np.atleast_2d(self.fourBar.getJointPts()[2]))
            self.wheelPts = np.array(np.atleast_2d(self.fourBar.getWheelPt()))
            self.wheelTravel = np.array([0])
            self.shockPts = np.array(np.atleast_2d(self.fourBar.getShockPt()))
            self.instantCenter = np.array(np.atleast_2d(self.fourBar.getInstantCenter()))
            self.fourBar.links[linkNum-1].rotate(angleIncrement)

            while True:
                self.fourBar.solve(1)
                self.joint1 = np.append(self.joint1, np.atleast_2d(self.fourBar.getJointPts()[1]), axis=0)
                self.joint2 = np.append(self.joint2, np.atleast_2d(self.fourBar.getJointPts()[2]), axis=0)
                self.wheelPts = np.append(self.wheelPts, np.atleast_2d(self.fourBar.getWheelPt()), axis=0)

                # self.wheelTravel = np.append(self.wheelTravel, np.linalg.norm(self.fourBar.getWheelPt() - self.wheelPts[0])) #true wheel travel
                self.wheelTravel = np.append(self.wheelTravel, self.fourBar.getWheelPt()[1] - self.wheelPts[0][1]) #vertical travel only
                
                self.shockPts = np.append(self.shockPts, np.atleast_2d(self.fourBar.getShockPt()), axis=0)
                self.instantCenter = np.append(self.instantCenter, np.atleast_2d(self.fourBar.getInstantCenter()), axis=0)

                self.fourBar.links[0].rotate(angleIncrement)
                
                if np.linalg.norm(self.fourBar.getShockPt() - self.shock.fixedPt) <= self.shock.length - self.shock.travel: #break when shock has reached max travel
                    break
        else:
            self.wheelPts = np.array(np.atleast_2d(self.singlePivotLink.getWheelVector().flatten() + self.mainPivot))
            self.wheelTravel = np.array([0])
            self.shockPts = np.array(np.atleast_2d(self.singlePivotLink.getShockVector().flatten() + self.mainPivot))
            self.instantCenter = np.array(np.atleast_2d(self.mainPivot))
            self.singlePivotLink.rotate(angleIncrement)
          
            while True:
                currentWheelPt = self.singlePivotLink.getWheelVector().flatten() + self.mainPivot
                self.wheelPts = np.append(self.wheelPts, np.atleast_2d(currentWheelPt), axis=0)
                self.wheelTravel = np.append(self.wheelTravel, currentWheelPt[1] - self.wheelPts[0][1])
                self.shockPts = np.append(self.shockPts, np.atleast_2d(self.singlePivotLink.getShockVector().flatten() + self.mainPivot), axis=0)
                self.instantCenter = np.append(self.instantCenter, np.atleast_2d(self.mainPivot), axis=0)

                self.singlePivotLink.rotate(angleIncrement)

                if np.linalg.norm(self.shockPts[len(self.shockPts)-1] - self.shock.fixedPt) <= self.shock.length - self.shock.travel: #break when shock has reached max travel
                    break

    def plotLinkage(self):
        if not self.singlePivot:
            self.fourBar.plot()

    def plotWheelPts(self): #use to plot on top of linkage figure
        plt.plot(*self.wheelPts.T, "--g")

    def plotInstantCenter(self):
        plt.plot(*self.instantCenter.T, "--r") #use to plot on top of linkage figure

    def plotAxlePath(self, axes):
        normalizedWheelPts = np.copy(self.wheelPts)
        normalizedWheelPts[:,0] -= normalizedWheelPts[0][0] #normalize wheel points to start at 0,0
        normalizedWheelPts[:,1] -= normalizedWheelPts[0][1]

        axes.plot(*normalizedWheelPts.T)
        axes.set_title("Axle Path")
        axes.set_xlabel("X (mm)")
        axes.set_ylabel("Y (mm)")
        axes.margins(0.05, 0.05)

    def plotLeverageRatio(self, axes):
        leverageRatio = np.zeros(len(self.wheelPts)-1)

        for i in range(len(self.wheelTravel) - 1):
            shockDelta = np.abs(np.linalg.norm(self.shockPts[i+1] - self.shock.fixedPt) - np.linalg.norm(self.shockPts[i] - self.shock.fixedPt))
            leverageRatio[i] = (self.wheelTravel[i+1] - self.wheelTravel[i]) / shockDelta #consider switch to vertical wheel travel only

        progressionPercent = 100 * (leverageRatio[0] - leverageRatio[len(leverageRatio)-1]) / leverageRatio[0]
        
        axes.plot(np.delete(self.wheelTravel,0), leverageRatio) #remove first entry in wheel travel array (0) to get lengths to line up
        axes.set_title("Leverage Ratio")
        axes.set_xlabel("Wheel Travel (mm)")
        axes.set_ylabel("Leverage Ratio")
        axes.text(0.95, 0.95,"Average Leverage Ratio: " + str(round(np.average(leverageRatio), 2)) + 
                  "\nProgression Percentage: " + str(round(progressionPercent, 2)) + "%"
                  , horizontalalignment='right', verticalalignment='top',transform=axes.transAxes)
        axes.margins(0, 0.15)

    def plotAntiRise(self, axes):
        antiRise = np.zeros(len(self.wheelPts))

        for i in range(len(self.wheelPts)):
            antiRise[i] = self.calculateAntiRise(self.wheelPts[i], self.instantCenter[i])

        axes.plot(self.wheelTravel, antiRise)
        axes.set_title("Anti-Rise")
        axes.set_xlabel("Wheel Travel (mm)")
        axes.set_ylabel("Anti Rise %")
        axes.margins(0, 0.2)
        axes.set_ybound(0)
        
    def calculateAntiRise(self, wheelPos, instantCenter):
        contactPt = np.array([wheelPos[0], wheelPos[1] - self.wheelDia/2]) #subtract wheel radius from y coordinate of axle point to get contact patch

        #get line between contact patch and instant center in form y = mx + b
        m = (instantCenter[1] - contactPt[1]) / (instantCenter[0] - contactPt[0])
        b = contactPt[1] - m * contactPt[0]

        antiRiseHeight = m * self.frontWheelPos[0] + b #height of intersection between contactPt-instant center line at front wheel x
        cgHeightDelta = self.cgHeight - contactPt[1] #vertical distance between cg height and contact point

        return 100 * (antiRiseHeight - contactPt[1])/cgHeightDelta #vertical distance between anti-rise intersection and contact patch divided by cg height

    def calculateAntiSquat(self, axlePt, instantCenter, chainRingTeeth, cassetteTeeth):

        chainRingRadius = 0.5 / np.sin(np.radians(180) / chainRingTeeth) / 2 * 25.4 #solve for sprocket pitch radius with tooth count 
        cassetteRadius = 0.5 / np.sin(np.radians(180) / cassetteTeeth) / 2 * 25.4

        #calculate approximate equation of chainline for fsolve guess
        approxChainRingPt = np.array([self.bbPos[0], self.bbPos[1] + chainRingRadius]) #approximate chain ring pt of tangency with point on chainring directly above bb
        approxCassetteRingPt = np.array([axlePt[0], axlePt[1] + cassetteRadius])

        approxA = - (approxCassetteRingPt[1] - approxChainRingPt[1]) / (approxCassetteRingPt[0] - approxChainRingPt[0]) # ax + y + c = 0, a is -1 * slope
        approxC = approxChainRingPt[1] - approxA * approxChainRingPt[0]

        #define system of 2 equations for tangency to chainring and cassette
        #each equation sets the distance from the line to the center point = radius of chainring/cassette
        def chainLineEqn(lineConsts): #solves for chainline equation in form ax + y = c
            return np.array([(np.abs(lineConsts[0]*self.bbPos[0] + self.bbPos[1] - lineConsts[1]) / np.sqrt(lineConsts[0] ** 2 + 1)) - chainRingRadius,
                             (np.abs(lineConsts[0]*axlePt[0] + axlePt[1] - lineConsts[1]) / np.sqrt(lineConsts[0] ** 2 + 1)) - cassetteRadius])
        
        chainLineConsts = scipy.optimize.fsolve(chainLineEqn, (approxA, approxC))

        #calculate line equation between rear axle and instant center in format ax + by = c
        a = instantCenter[1] - axlePt[1] #y2 - y1
        b = axlePt[0] - instantCenter[0] #x1 - x2
        c = axlePt[0] * (instantCenter[1] - axlePt[1]) - axlePt[1] * (instantCenter[0] - axlePt[0]) #x1 * (y2-y1)- y1 * (x2-x1)

        #solve for instantaneous force center (IFC) as intersection between chainline and axle-IC line
        det = chainLineConsts[0] * b - a #b1 = 1 since chainline is in form ax + 1*y = c

        IFCx = (chainLineConsts[1] * b - c) / det
        IFCy = (chainLineConsts[0] * c - a * chainLineConsts[1]) / det

        #solve for line between rear contact patch and IFC (anti-squat force line) in form y = mx + b
        contactPt = np.array([axlePt[0], axlePt[1] - self.wheelDia/2])
        ASLineSlope = (IFCy - contactPt[1]) / (IFCx - contactPt[0])
        ASLineIntercept = contactPt[1] - ASLineSlope * contactPt[0]

        #solve for anti-squat value
        ASHeight = ASLineSlope * self.frontWheelPos[0] + ASLineIntercept - contactPt[1] #vertical distance between anti-squat force line intercept w/ front wheel x pos and contact point
        CGHeight = self.cgHeight - contactPt[1] #vertical distance between contact point and cg
        
        return 100 * ASHeight / CGHeight
    
    def plotAntiSquat(self, axes):
        chainRingTeeth = 32

        cassetteTeeth = np.array([11, 24, 52])

        antiSquatHigh = np.zeros(len(self.wheelPts))
        antiSquatMid = np.zeros(len(self.wheelPts))
        antiSquatLow = np.zeros(len(self.wheelPts))

        for i in range(len(self.wheelPts)):
            antiSquatHigh[i] = self.calculateAntiSquat(self.wheelPts[i], self.instantCenter[i], chainRingTeeth, cassetteTeeth[0])

        for i in range(len(self.wheelPts)):
            antiSquatMid[i] = self.calculateAntiSquat(self.wheelPts[i], self.instantCenter[i], chainRingTeeth, cassetteTeeth[1])
            
        for i in range(len(self.wheelPts)):
            antiSquatLow[i] = self.calculateAntiSquat(self.wheelPts[i], self.instantCenter[i], chainRingTeeth, cassetteTeeth[2])
        
        axes.plot(self.wheelTravel, antiSquatHigh, "mediumseagreen", label= str(chainRingTeeth) + "T / " + str(cassetteTeeth[0]) + "T")
        axes.plot(self.wheelTravel, antiSquatMid, "orange", label= str(chainRingTeeth) + "T / " + str(cassetteTeeth[1]) + "T")
        axes.plot(self.wheelTravel, antiSquatLow, "red", label= str(chainRingTeeth) + "T / " + str(cassetteTeeth[2]) + "T")
        axes.legend()
        axes.set_title("Anti-Squat")
        axes.set_xlabel("Wheel Travel (mm)")
        axes.set_ylabel("Anti Squat %")
        axes.margins(0, 0.2)
        axes.set_ybound(0)
        
    def plotKinematics(self):
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 8))
        self.plotAxlePath(ax1)
        self.plotLeverageRatio(ax2)
        self.plotAntiRise(ax3)
        self.plotAntiSquat(ax4)
        ax1.grid(linewidth=0.2)
        ax2.grid(linewidth=0.2)
        ax3.grid(linewidth=0.2)
        ax4.grid(linewidth=0.2)
        plt.tight_layout()
        plt.show()



