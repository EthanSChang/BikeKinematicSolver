import numpy as np
import matplotlib.pyplot as plt
from Link import Link
from shock import Shock
from fourBar import FourBar
from linkage import Linkage


#numbers for raaw madonna v3
startPt = np.array([18.255, 243.728])
endPt = np.array([-19.655, 70.706])
joint1Pt = np.array([-98.04, 251.689])
joint2Pt = np.array([-405.8, 26.92])

shockMountingPt = np.array([38.605, 93.299])
shock = Shock(210, 65, shockMountingPt)

link1 = Link(np.array([startPt, joint1Pt]), shockPoint=np.array([79.758, 294.125]))
link2 = Link(np.array([joint1Pt, joint2Pt]), wheelPoint=np.array([-460.026, 31.466]))
link3 = Link(np.array([joint2Pt, endPt]))
links = np.array([link1, link2, link3])

fourBar = FourBar(startPt, links, endPt)
angleIncrement = np.radians(-0.05)

wheelDia = 29*25.4
frontWheelPos = np.array([818.513, 25.114])
cgHeight = 750 #relative to same coordinate frame given in links
bbPos = np.array([0,0])

horstLink = Linkage(wheelDia, frontWheelPos, cgHeight, bbPos, shock, fourBar=fourBar)

horstLink.solveLinkage(1, angleIncrement)
# testLinkage.plotLinkage()
# testLinkage.plotWheelPts()
# testLinkage.plotInstantCenter()
# plt.show()

horstLink.plotKinematics()



