#!/usr/bin/python
# The file demonstrates:
#   1. Adding Obstacles to the environment (refer to buildWorld.py as well)
#   2. Setting up a robot
#   3. Perform collision checking
#   4. Modify the robot configurations and visualize
#   5. Adding text objects and modifying them

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import time
import math
import addToEnv as ate
sys.path.append("./kinematics/")
from cube6 import cube6DoF
from decimal import Decimal
from sphericalCollisionChecker import SphericalCollisionChecker

import RRT as rrt

if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: simulation.py [world_file]"
        exit()

    #Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    # Add obstacles in the environment
    obsPosList, obsDimList = ate.getCubeObs(world, 10)
    
    # Define Bounding Sphere collsion checker
    scc = SphericalCollisionChecker(obsPosList, obsDimList)
    spherePosList = scc.getSpheres()

    # Uncomment the next line to see the bounding spheres
    #ate.getSphereObs(world,spherePosList, obsDimList)

    
    robot = cube6DoF(world.robot(0), "satellite", vis)

    trace = rrt.rrt_bidirectional(world,robot,scc)
    if trace!=None:
        trace = list(trace)
        #ate.getPath(world, trace)
        #vis.add("world",world)

    # Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)
    
    #robot = cube6DoF(world.robot(0), "satellite", vis)

    # Display the world coordinate system
    vis.add("WCS", [so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)


    #vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)

    # On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    print "Starting visualization window#..."

    # Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    '''
    # Calling RRT
    trace = rrt.rrt_bidirectional(world,robot,scc)
    if trace!=None:
        trace = list(trace)
        ate.getPath(world, trace)
       # vis.add("world",world)
    '''

    vis.show()
    simTime = 300
    startTime = time.time()
    oldTime = startTime
    while vis.shown() and (time.time() - startTime < simTime):
        vis.lock()
        # You may modify the world here.
        # Specifying change in configuration of the robot

        # 6DoF cube robot
        q = robot.getConfig()
        q[0] = 0
        q[1] = 0
        q[2] = 0
        q[3] = 0
        q[4] = 0
        q[5] = 0
        robot.setConfig(q)

        vis.unlock()
        
        # Set collision flag false for each iteration
        collisionFlag = False
        # Trace path
    
        if trace!=None:
            for i in range(len(trace)):
                vis.lock()
                tpos = trace[i]
                rpos = robot.getConfig()
                rpos[0] = tpos[0]
                rpos[1] = tpos[1]
                rpos[2] = tpos[2]
                rpos[3] = tpos[3]
                rpos[4] = tpos[4]
                rpos[5] = tpos[5]

                robot.setConfig(rpos)
                q = robot.getConfig()
                ate.getPath(world, q)
                vis.add("world", world)
                q2f = [ '{0:.2f}'.format(elem) for elem in q]
                strng = "Robot configuration: " + str(q2f)
                if scc.checkCollision(q):
                    collisionFlag = True
                    strng = "Robot collides with Obstacle"
                    print(strng)
                    vis.addText("textCol", strng)
                    vis.setColor("textCol", 0.8500, 0.3250, 0.0980)

                '''
                collRT0 = collisionChecker.robotObjectCollisions(world.robot(0))
                for i,j in collRT0:
                    collisionFlag = True
                    strng = "Robot collides with "+j.getName()
                    print(strng)
                    vis.addText("textCol", strng)
                    vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
                    break
                '''
                vis.addText("textConfig", strng)
                time.sleep(0.01)
                vis.unlock()
        
        # Changes to the visualization must be done outside the lock
        vis.setColor("textCol", 0.4660, 0.6740, 0.1880)
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
