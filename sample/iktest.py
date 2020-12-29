from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from numpy import *
import math
import time

robotItem = None
for bodyItem in RootItem.instance.getDescendantItems(BodyItem):
    legged = LeggedBodyHelper(bodyItem.body)
    if legged.numFeet == 2:
        robotItem = bodyItem
        break

if not robotItem:
    print("A biped robot item is not found.")
    exit()
    
robot = robotItem.body
waist = robot.rootLink
foot1 = legged.footLink(0)
foot2 = legged.footLink(1)
waistToFoot1 = JointPath.getCustomPath(robot, waist, foot1)
waistToFoot2 = JointPath.getCustomPath(robot, waist, foot2)

def doIK(jointPath):
    print("Joint path from {0} to {1} {2} a custom IK."
          .format(jointPath.baseLink.name, jointPath.endLink.name, "has" if jointPath.hasCustomIK() else "does not have"))
    T0 = jointPath.endLink.T.copy()
    for i in range(0, 361, 5):
        dx = 0.1 * math.sin(math.radians(i))
        dy = 0.1 * math.sin(math.radians(i))
        dz = 0.1 * (1.0 - math.cos(math.radians(i * 2)))
        yaw = math.sin(math.radians(i))
        T = T0.copy()
        # translation
        T[0, 3] += dx
        T[1, 3] += dy
        T[2, 3] += dz
        # rotation
        T[0:3, 0:3] = rotFromRpy([0, 0, yaw])
        result = jointPath.calcInverseKinematics(T)
        if not result:
            rpy = degrees(rpyFromRot(T[0:3, 0:3]))
            print("IK error for position ({:.3f}, {:.3f}, {:.3f}, {:.1f}, {:.1f}, {:.1f})"
                  .format(p[0], p[1], p[2], rpy[0], rpy[1],rpy[2]))
        robotItem.notifyKinematicStateChange()
        updateGui()
        time.sleep(0.01)

doIK(waistToFoot1)
doIK(waistToFoot2)
