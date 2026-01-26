import numpy as np
import matplotlib.pyplot as plt
from Link import Link
from shock import Shock
from fourBar import FourBar
from linkage import Linkage


#numbers for raaw madonna v2 in linkage x3 demo
startPt = np.array([8.7, 272.4])
endPt = np.array([4.5, 63.1])
joint1Pt = np.array([-131, 301.1])
joint2Pt = np.array([-389.1, 26.5])

shockMountingPt = np.array([48.6, 122.9])
shock = Shock(210, 65, shockMountingPt)

link1 = Link(np.array([startPt, joint1Pt]), shockPoint=np.array([80.6, 325.6]))
link2 = Link(np.array([joint1Pt, joint2Pt]), wheelPoint=np.array([-450.1, 33.4]))
link3 = Link(np.array([joint2Pt, endPt]))
links = np.array([link1, link2, link3])

fourBar = FourBar(startPt, links, endPt)
angleIncrement = np.radians(-0.05)

wheelDia = 29*25.4
frontWheelPos = np.array([893.8, 33.4])
cgHeight = 600.1 #relative to same coordinate frame given in links
bbPos = np.array([0,0])

horstLink = Linkage(wheelDia, frontWheelPos, cgHeight, bbPos, shock, fourBar=fourBar)

# horstLink.plotLinkage()
horstLink.solveLinkage(1, angleIncrement)

# horstLink.plotWheelPts()
# horstLink.plotInstantCenter()
# plt.show()

horstLink.plotKinematics()


#numbers for starling mega murmur
# singlePivotLink = Link(np.array([[13.887, 73.289], [-463.026, 16.438]]), wheelPoint=np.array([-463.026, 16.438]), shockPoint=np.array([-15.356, 266.199]))
# mainPivotPt = np.array([13.887, 73.289])
# wheelDia = 29*25.4
# frontWheelPos = np.array([850.492, 32.557])
# cgHeight = 670 #relative to same coordinate frame given in links
# bbPos = np.array([0,0])
# shockMountingPt = np.array([212.966, 319.093])
# shock = Shock(230, 60, shockMountingPt)

# singlePivot = Linkage(wheelDia, frontWheelPos, cgHeight, bbPos, shock, link=singlePivotLink, mainPivot=mainPivotPt)
# singlePivot.solveLinkage(1, np.radians(-0.05))
# singlePivot.plotKinematics()