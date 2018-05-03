# -*- coding: utf-8 -*-

"""
Created on Mon Apr 16 21:21:56 2018

@author: vinay

Implementation of Bidirectional RRT
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import mathUtils
from klampt.math import so3
from klampt import *
import klampt.model.collide as collide
import math

# Environment limits
env_limits = np.array([-2, +2, -2, +2, 0, +2]).reshape(3,2)

# Robot dimensions
robot_length = 0.05 
robot_height = 0.05
robot_width = 0.05

# Start and goal configurations
xinit = np.array([2, -2, 2, 0, 0, 0])
xgoal = np.array([0, 2, 0, math.pi, math.pi/2, math.pi/3])

# Max iterations
maxiter = 10000

# Max distance between nodes
epsilon = 0.01
orientEpsilon = 0.01

# Tree data structure
rrt_init = []
rrt_goal = []

# Class defining a node and its connected edges in rrt
class Node:
    def __init__(self, coords):
        self.coords = coords
        self.parent = None
        
    def get_parent(self):
        return self.parent
    
    def get_coords(self):
        return self.coords
    
    def set_parent(self, x):
        self.parent = x
        return
    
def set_x_to_q(q, x):
    q[0] = x[0]
    q[1] = x[1]
    q[2] = x[2]
    q[3] = x[3]
    q[4] = x[4]
    q[5] = x[5]
    return q

# Klampt collision checking - Not using
def checkCollision(world, robot, q):
    robot.setConfig(q)
    collisionChecker = collide.WorldCollider(world)
    coll = collisionChecker.robotObjectCollisions(world.robot(0))
    for i,j in coll:
        return True
    return False


# Creates a new random point in the environment
def new_point():
    xlim = env_limits[0]
    ylim = env_limits[1]
    zlim = env_limits[2]
    
    xrand = np.random.uniform(xlim[0], xlim[1])
    yrand = np.random.uniform(ylim[0], ylim[1])
    zrand = np.random.uniform(zlim[0], zlim[1])

    zangle = np.random.uniform(0, math.pi)
    yangle = np.random.uniform(0, math.pi)
    xangle = np.random.uniform(0, math.pi)

    
    # Checking collision in rrt_bidirectional method
    return np.array([xrand, yrand, zrand, zangle, yangle, xangle])

# Distance between two orientations
def orientation_dist(orient1, orient2):
    rotMat1 = mathUtils.euler_zyx_mat(orient1)
    rotMat2 = mathUtils.euler_zyx_mat(orient2)
    dist = so3.distance(rotMat1, rotMat2)
    return dist

# Find nearest neighbor in the the given tree to x
def nearest_neighbor(x, rrt):
    for index in range(len(rrt)):
        node = rrt[index]
        coord = node.get_coords()
        dist = np.sqrt((x[0]-coord[0])**2 + (x[1]-coord[1])**2 + (x[2]-coord[2])**2)
        orientDist = orientation_dist([x[3],x[4],x[5]],[coord[3],coord[4],coord[5]])
        if(index==0):
            minDist = dist+orientDist
            minDistIndex = index
        else:
            if(dist<minDist):
                minDist = dist+orientDist
                minDistIndex = index
    return minDistIndex

# Generate new state in the direction of the random position x
def new_state(world, robot, scc, x, xnear):
    dist = np.sqrt((x[0]-xnear[0])**2 + (x[1]-xnear[1])**2 + (x[2]-xnear[2])**2)
    orientdist = orientation_dist([x[3],x[4],x[5]],[xnear[3],xnear[4],xnear[5]])
    if(dist<=epsilon and orientdist<=orientEpsilon):
        return x, 0
    else:
        ratio = epsilon/dist
        orientratio = orientEpsilon/orientdist
        newxcoord = (1-ratio)*xnear[0]+ratio*x[0]
        newycoord = (1-ratio)*xnear[1]+ratio*x[1]
        newzcoord = (1-ratio)*xnear[2]+ratio*x[2]
        newzangle = (1-orientratio)*xnear[3]+orientratio*x[3]
        newyangle = (1-orientratio)*xnear[4]+orientratio*x[4]
        newxangle = (1-orientratio)*xnear[5]+orientratio*x[5]
        xnew = [newxcoord, newycoord, newzcoord, newzangle, newyangle, newxangle]
        # Checking for collisions
        '''
	#Klampt collsion checker
        if checkCollision(world, robot, xnew):
            return None, -1
        '''
        if scc.checkCollision(xnew):
            return None, -1
        return xnew, 1

# Extend the tree in the direction of x
def extend(world, robot, scc, x, rrtIndicator):
    if rrtIndicator == 0:
        nearIndex = nearest_neighbor(x, rrt_init)
        nearestNode = rrt_init[nearIndex]
        xnew, status = new_state(world, robot, scc, x, nearestNode.get_coords())
        if status == -1:
            return None, -1
        newNode = Node(xnew)
        newNode.set_parent(nearestNode)
        rrt_init.append(newNode)
    else:
        nearIndex = nearest_neighbor(x, rrt_goal)
        nearestNode = rrt_goal[nearIndex]
        xnew, status = new_state(world, robot, scc, x, nearestNode.get_coords())
        if status == -1:
            return None, -1
        newNode = Node(xnew)
        newNode.set_parent(nearestNode)
        rrt_goal.append(newNode)
    return xnew, status

# Compare two floating points
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# Compare two 3d coordinate points
def compare_coordinates(coord1, coord2):
    if(isclose(coord1[0], coord2[0]) and isclose(coord1[1], coord2[1]) and isclose(coord1[2], coord2[2])):
        return True
    else:
        return False

# Compare two nodes
def compare_nodes(n1, n2):
    coord1 = n1.get_coords()
    coord2 = n2.get_coords()
    flag = False
    if compare_coordinates(coord1, coord2) and (orientation_dist([coord1[3],coord1[4],coord1[5]],[coord2[3],coord2[4],coord2[5]])<=orientEpsilon):
        flag = True
    return flag

# Plot the path in using matplotlib
def display(path):
    path = np.array(path)
    positions = path[:,0:3]
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(*zip(*positions))
    ax.plot(np.array([xinit[0],xgoal[0]]), np.array([xinit[1],xgoal[1]]), np.array([xinit[2],xgoal[2]]), 'ro')
    plt.show()

# Gives final path from init to goal
def get_path(x):
    lastNode_rrt_init = rrt_init[len(rrt_init)-1]
    lastNode_rrt_goal = rrt_goal[len(rrt_goal)-1]
    
    trace_rrt_init = []
    #trace_rrt_goal = []
    trace_final = []
    
    if(compare_nodes(lastNode_rrt_init, lastNode_rrt_goal)):
        parent = lastNode_rrt_init.get_parent()
        while(parent!=None and not compare_coordinates(parent.get_coords(), xinit)):
            trace_rrt_init.append(parent.get_coords())
            parent = parent.get_parent()
        trace_rrt_init.append(xinit)
        
        for i in reversed(trace_rrt_init):
            trace_final.append(i)
        
        trace_final.append(lastNode_rrt_init.get_coords())
            
        parent = lastNode_rrt_goal.get_parent()
        while(parent!=None and not compare_coordinates(parent.get_coords(), xgoal)):
            trace_final.append(parent.get_coords())
            parent = parent.get_parent()
        trace_final.append(xgoal)
    return trace_final

# Implementation of Bidriectional RRT
def rrt_bidirectional(world, robot, scc):
    
    q = robot.getConfig()
    q = set_x_to_q(q, xinit)
    '''
    if checkCollision(world, robot, q):
        return None
    '''
    if scc.checkCollision(q):
        return None

    q = set_x_to_q(q, xgoal)
    '''
    if checkCollision(world, robot, q):
        return None
    '''
    if scc.checkCollision(q):
        return None

    rrt_init.append(Node(xinit))
    rrt_goal.append(Node(xgoal))
    
    a = 0
    b = 1
    trace = None
    count = 0
    for k in range(maxiter):
        if count>1000:
            print("************************")
            print("TRAPPED. Could not extend the tree since 1000 iterations")
            break
        xrand = new_point()
        if scc.checkCollision(xrand):
            continue
        # Status 0 if reached, 1 if advanced, -1 if trapped
        xnew, status1 = extend(world, robot, scc, xrand, a)
        if status1 == -1:
            count += 1
            continue
        _, status2 = extend(world, robot, scc, xnew, b)
        if status2 == -1:
            count += 1
            continue
        count = 0
        if status2 == 0:
            print("goal reached at iteration:",k)
            trace = get_path(xnew)
            break
        a = a + b
        b = a - b
        a = a - b
    if(trace!=None):
        display(trace)
        print(trace)
    else:
        print("could not find a path")
    return trace 
    
