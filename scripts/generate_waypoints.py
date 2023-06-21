#!/usr/bin/env python


import rospy
# from cv_bridge import CvBridge

from uav_sim.srv import *
import numpy as np
from numpy.linalg import norm

# import rospkg

def get_rand_position():
    radius = 100
    randTheta = np.random.random()*2*np.pi
    randSign = np.random.choice([-1,1])
    xPos = radius*np.cos(randSign*randTheta)
    yPos = radius*np.sin(randSign*randTheta)
    return xPos, yPos

def get_values(req):
    # print("here in callback")
    # req.numberVehicles

    allVehicles = []
    allPositions = []

    xPos, yPos = get_rand_position()
    allPositions.append(np.array([xPos,yPos]))

    for i in range(req.numberVehicles-1):

        foundOther = True
        while foundOther:
            xPos, yPos = get_rand_position()
            for pos in allPositions:
                if norm(pos-np.array([xPos,yPos])) < req.radius*2.1:
                    # pdb.set_trace()
                    foundOther = True
                    break
                foundOther = False

        allPositions.append(np.array([xPos,yPos]))

    x = []
    y = []
    z = []
    dir = []
    for pos in allPositions:
        x.append(pos[0])
        x.append(-pos[0])
        # x.append(pos[0])
        # x.append(-pos[0])

        y.append(pos[1])
        y.append(-pos[1])
        # y.append(pos[1])
        # y.append(-pos[1])

        z.append(0)
        z.append(0)
        # z.append(0)
        # z.append(0)

        dir.append(0)
        dir.append(0)
        # dir.append(0)
        # dir.append(0)

    # print(x,y,z)

    return WaypointsResponse(x,y,z,dir)


def waypoint_server():
    rospy.init_node('waypoint_server')
    s = rospy.Service('/waypoints', Waypoints, get_values)
    print("Waypoint Server Ready")
    rospy.spin()


if __name__ == "__main__":
    waypoint_server()
