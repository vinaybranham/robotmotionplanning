# Created by : vinay
# Collision Checker class based on bounding spheres
# Takes in the configurations(positions and orientations) of the obstacles and robot and their dimensions
# Approximates any obstacle to a rectangular parallelepiped based on the dimensions
# Bounds them with sphere and checks for collisions

import numpy as np
from klampt.math import so3
import math
import mathUtils

class SphericalCollisionChecker:

    def __init__(self, obsPosList, obsDimList):
        self.obsList = obsPosList
        self.obsDimList = obsDimList
        self.spherePosList = []

    #Calculates Radius of bounding sphere based on dimensions
    def getRadius(self, dim):
        return np.sqrt(dim[0]**2+dim[1]**2+dim[2]**2)/2

    #Takes the configuration of robot as input
    #Bounds the robot and obstacles with sphere and
    #Checks for collision based on distance between the centers of spheres
    def checkCollision(self, q):
        collisionFlag = False
        # Hardcoded robot dimensions
        robDim = [0.05,0.05,0.05]
        robSphereRad = self.getRadius(robDim)
        rotMat = mathUtils.euler_zyx_mat([q[3],q[4],q[5]])
        # position is half of the robots dimension - Center of the approximated robot when placed at [0,0,0]
        robSphere = so3.apply(rotMat, [robDim[0]/2,robDim[1]/2,robDim[2]/2])
        robSphere = np.array(robSphere)
        robSphere = robSphere + [q[0],q[1],q[2]]

        if len(self.spherePosList)==len(self.obsList):
            for i in range(len(self.spherePosList)):
                obsDim = self.obsDimList[i]
                obsSphereRad = self.getRadius(obsDim)
                obsSphere = self.spherePosList[i]
                dist = np.sqrt((obsSphere[0]-robSphere[0])**2 + (obsSphere[1]-robSphere[1])**2 + (obsSphere[2]-robSphere[2])**2)
                if(dist <= robSphereRad+obsSphereRad):
                    collisionFlag = True
                    break
            return collisionFlag

        for i in range(len(self.obsList)):
            obsDim = self.obsDimList[i]
            obsSphereRad = self.getRadius(obsDim)
            pos = self.obsList[i]
            rotMat = mathUtils.euler_zyx_mat([pos[3],pos[4],pos[5]])
            obsSphere = so3.apply(rotMat, [obsDim[0]/2,obsDim[1]/2,obsDim[2]/2])
            obsSphere = np.array(obsSphere)
            obsSphere = obsSphere + [pos[0],pos[1],pos[2]]
            dist = np.sqrt((obsSphere[0]-robSphere[0])**2 + (obsSphere[1]-robSphere[1])**2 + (obsSphere[2]-robSphere[2])**2)
            if(dist <= robSphereRad+obsSphereRad):
                collisionFlag = True
                break
        return collisionFlag

    # Gets the centers of the approximated obstacles
    def getSpheres(self):
        for i in range(len(self.obsList)):
            obsDim = self.obsDimList[i]
            pos = self.obsList[i]
            rotMat = mathUtils.euler_zyx_mat([pos[3],pos[4],pos[5]])
            obsSphere = so3.apply(rotMat, [obsDim[0]/2,obsDim[1]/2,obsDim[2]/2])
            obsSphere = np.array(obsSphere)
            obsSphere = obsSphere + [pos[0],pos[1],pos[2]]
            self.spherePosList.append(obsSphere)
        return self.spherePosList
