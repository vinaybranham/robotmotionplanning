#!/usr/bin/python
# Created by : vinay
# The functions are for creating additional components for the environment
#  1. getCube: Get the geometry of a cube based on given dimensions
#  2. getCubeObs: Add some cube obstacles to the environment based on the count
#  3. getSphere: Get the geometry of a sphere based on given dimensions
#  4. getSphereObs: Add some cube obstacles to the environment based on the positions and dimensions
import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import time
import math
import mathUtils
import numpy as np

def getCube(dimX = 0.35, dimY = 0.35, dimZ = 0.35, pos=[0, 0, 0, 0, 0, 0]):
    cube = Geometry3D()
    cube.loadFile("cube.off")
    cube.scale(dimX,dimY,dimZ)
    rotMat = mathUtils.euler_zyx_mat([pos[3],pos[4],pos[5]])
    cube.transform(rotMat,[pos[0], pos[1], pos[2]])
    return cube, pos, [dimX,dimY,dimZ]

def getSphere(dimX, dimY, dimZ, pos=[0, 0, 0, 0, 0, 0]):
    sphere = Geometry3D()
    sphere.loadFile("sphere.off")
    sphere.scale(dimX, dimY, dimZ)
    rotMat = mathUtils.euler_zyx_mat([pos[3],pos[4],pos[5]])
    sphere.transform(rotMat,[pos[0], pos[1], pos[2]])
    return sphere

# Generate cube Obstacles
def getCubeObs(world, count = 10):
    obsGeo = Geometry3D()
    obsGeo.setGroup()
    #cubeList = []
    posList = []
    dimList = []
    for i in range(count):
        x = np.random.uniform(-2,2)
        y = np.random.uniform(-2,2)
        z = np.random.uniform(0,2)
        za = np.random.uniform(0,math.pi)
        ya = np.random.uniform(0,math.pi)
        xa = np.random.uniform(0,math.pi)
        cube, pos, dim = getCube(pos=[x,y,z,za,ya,xa])
        posList.append(pos)
        dimList.append(dim)
        obsGeo.setElement(i,cube)
    '''
    cube, pos, dim = getCube(pos=[0,0,0.05,0,0,0])
    posList.append(pos)
    dimList.append(dim)
    obsGeo.setElement(0,cube)
    '''
    obs_setup = world.makeRigidObject("OBS")
    obs_setup.geometry().set(obsGeo)
    return posList, dimList

# Generate sphere Obstacles
def getSphereObs(world, posList, dimList):
    obsGeo = Geometry3D()
    obsGeo.setGroup()
    #cubeList = []
    #for i in range(count):
    #    x = np.random.uniform(-2,2)
    #    y = np.random.uniform(-2,2)
    #    z = np.random.uniform(0,2)
    #    za = np.random.uniform(0,math.pi)
    #    ya = np.random.uniform(0,math.pi)
    #    xa = np.random.uniform(0,math.pi)
    #    cube = getCube([x,y,z,za,ya,xa])
    #    obsGeo.setElement(i,cube)
    for i in range(len(posList)):
        pos = posList[i]
        dim = dimList[i]
        sphere = getSphere(dim[0], dim[1], dim[2], [pos[0],pos[1],pos[2],0,0,0])
        obsGeo.setElement(i,sphere)
    '''
    rotMat = mathUtils.euler_zyx_mat([math.pi/4,math.pi/6,math.pi/3])
    p = so3.apply(rotMat,[0.125,0.125,0.125])
    p = [0.025,0.025,0.025]
    p = np.array(p)
    #p = p+[1,1,1]
    sphere = getSphere([p[0],p[1],p[2],0,0,0])
    obsGeo.setElement(0,sphere)
    '''
    obs_setup = world.makeRigidObject("SphereOBS")
    obs_setup.geometry().set(obsGeo)

def getPath(world, pos, dim=[0.005,0.005,0.005]):
    obsGeo = Geometry3D()
    obsGeo.setGroup()
    #cubeList = []
    #for i in range(count):
    #    x = np.random.uniform(-2,2)
    #    y = np.random.uniform(-2,2)
    #    z = np.random.uniform(0,2)
    #    za = np.random.uniform(0,math.pi)
    #    ya = np.random.uniform(0,math.pi)
    #    xa = np.random.uniform(0,math.pi)
    #    cube = getCube([x,y,z,za,ya,xa])
    #    obsGeo.setElement(i,cube)
    sphere = getSphere(dim[0], dim[1], dim[2], [pos[0],pos[1],pos[2],0,0,0])
    obsGeo.setElement(0,sphere)
    '''
    rotMat = mathUtils.euler_zyx_mat([math.pi/4,math.pi/6,math.pi/3])
    p = so3.apply(rotMat,[0.125,0.125,0.125])
    p = [0.025,0.025,0.025]
    p = np.array(p)
    #p = p+[1,1,1]
    sphere = getSphere([p[0],p[1],p[2],0,0,0])
    obsGeo.setElement(0,sphere)
    '''
    obs_setup = world.makeRigidObject("Path")
    obs_setup.geometry().set(obsGeo)
    obs_setup.appearance().setColor(0.75,0.5,0.25,1)

