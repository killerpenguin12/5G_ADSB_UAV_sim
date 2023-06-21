#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm

import pdb
import warnings

import rospy
from uav_sim.srv import *

collisionRadius = 2.0

def get_angle(vect1, vect2):
    atanA = np.arctan2(vect1[0],vect1[1])
    atanB = np.arctan2(vect2[0],vect2[1])
    return atanA - atanB


def get_angle_between(vect1,vect2):
    return np.arccos(check_arcs(np.dot(vect1,vect2)/(norm(vect1)*norm(vect2))))

def get_intersect(p1,p2,q1,q2):
    ## Referece:
    ## https://blogs.sas.com/content/iml/2018/07/09/intersection-line-segments.html
    A = np.column_stack((p2-p1,q1-q2))
    B = q1-p1
    if not np.linalg.det(A) == 0.0:
        x = np.linalg.solve(A,B)
        if all(x >= 0) and all(x <= 1):
            # pdb.set_trace()
            return (1-x[0])*p1+x[0]*p2

    return np.array([])


def get_RVO_triangle(av1Xo, av1Vo, av2Xo, av2Vo, futureVO=False, CC=False):

    x_pos, y_pos = av1Xo
    x_vel, y_vel = av1Vo
    xi_pos, yi_pos = av2Xo
    xi_vel, yi_vel = av2Vo
    # if futureVO:
    #     direction = av2.nextWP - av2.currentWP
    #     direction = direction / norm(direction)
    #     speed = norm(av2Vo)
    #     xi_vel, yi_vel = direction * speed
    Xr = av2Xo - av1Xo
    xr_pos, yr_pos = Xr

    radius = collisionRadius * 2.0
    # if all(abs(av1Xo - av2Xo) < collisionRadius * 2.0):
    #     radius = collisionRadius

    trans_x = 0.0
    trans_y = 0.0
    alpha = 1/2.0
    if not CC:
        trans_x = (x_vel + xi_vel)/2.0
        trans_y = (y_vel + yi_vel)/2.0

        # trans_x = (1-alpha)*x_vel + alpha*xi_vel
        # trans_y = (1-alpha)*y_vel + alpha*yi_vel
        # trans_x = xi_vel
        # trans_y = yi_vel

    distR = norm(Xr)

    distA = np.sqrt(distR**2+radius**2)

    # epsilon = 0.01
    # with warnings.catch_warnings():
    #     warnings.filterwarnings('error')
    #     try:
    theta = np.arcsin(check_arcs(radius/distR))
        # except Warning as e:
        #     print(radius/distR, check_arcsin(radius/distR))


    theta2 = np.arctan2(yr_pos,xr_pos)
    rotate = np.matrix([[np.cos(theta2), np.sin(theta2)],
                        [-np.sin(theta2), np.cos(theta2)]])

    ptB = np.array([x_pos+distA*np.cos(theta), y_pos+distA*np.sin(theta)])
    ptB = ptB - av1Xo
    ptB = ptB * rotate
    ptB = ptB / norm(ptB)
    ptB = ptB * (norm(Xr)+norm(av2Vo))*5

    ptC = np.array([x_pos+distA*np.cos(theta), y_pos-distA*np.sin(theta)])
    ptC = ptC - av1Xo
    ptC = ptC * rotate
    ptC = ptC / norm(ptC)
    ptC = ptC * (norm(Xr)+norm(av2Vo))*5


    xy = [[x_pos+trans_x, y_pos+trans_y],
        [x_pos+ptB[0,0]+trans_x, y_pos+ptB[0,1]+trans_y],
        [x_pos+ptC[0,0]+trans_x, y_pos+ptC[0,1]+trans_y]]


    return xy

def check_encounter(av1,rvoTri):

    # rvoTri = get_RVO_triangle(av1, av2)
    # Compute vectors
    A = np.array(rvoTri[0])
    B = rvoTri[1]
    C = rvoTri[2]
    P = av1Xo + av1VoD
    return barycentric(A, B, C, P)

def barycentric(A, B, C, P):
    ## Barycentric Technique: http://blackpawn.com/texts/pointinpoly/
    ## For checkking point within a triangle

    v0 = C - A
    v1 = B - A
    v2 = P - A

    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    if (dot00 * dot11 - dot01 * dot01) == 0.0:
        # pdb.set_trace()
        return False
    # with warnings.catch_warnings():
    #     warnings.filterwarnings('error')
    #     try:
    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * invDenom
    v = (dot00 * dot12 - dot01 * dot02) * invDenom
        # except Warning as e:
        #     pdb.set_trace()
        #     print("here")


    return (u >= 0.001) and (v >= 0.001) and (u + v < 1.0)



def check_arcs(input):
    if input < -1:
        return -1
    elif input > 1:
        return 1
    return input


def get_rotation_matrix(theta):
    rotation = np.matrix([[np.cos(theta), np.sin(theta)],
                        [-np.sin(theta), np.cos(theta)]])
    return rotation

def same_velocity_closest_pt(A,B,C,P,O):
    theta = 0.0

    while barycentric(A,B,C,P):
        theta = theta +  0.1
        pt = P-O
        pt = pt * get_rotation_matrix(theta*np.pi/180)
        pt = np.squeeze(np.asarray(pt))
        P = pt+O
        if abs(theta) < 0.00001:
            pdb.set_trace()

    ptAB = P
    theta = -theta
    # pt = P-O
    pt = pt * get_rotation_matrix(theta*np.pi/180)
    pt = np.squeeze(np.asarray(pt))
    P = pt+O

    while barycentric(A,B,C,P):
        theta = theta - 0.1
        pt = P-O
        pt = pt * get_rotation_matrix(theta*np.pi/180)
        pt = np.squeeze(np.asarray(pt))
        P = pt+O
        if abs(theta) < 0.00001:
            pdb.set_trace()

    ptAC = P

    return [ptAB, ptAC]

def get_points_along_line(pt1, pt2, numPoints=100):
    A = np.linspace(pt1[0], pt2[0], numPoints)
    B = np.linspace(pt1[1], pt2[1], numPoints)
    return np.column_stack((A,B))


def get_closest_pt(A, B, Pt):
    ## https://blender.stackexchange.com/questions/94464/finding-the-closest-point-on-a-line-defined-by-two-points
    n = (B - A) / norm(B - A)
    ap = Pt - A
    t = np.dot(ap,n)
    x =  A + t * n #  x is a point on line

    return x

def get_admissible_velocities(AoMax,N=4):
    grid = []
    distanceAway = 0.001 * AoMax
    radius = np.linspace(0,distanceAway,N)
    numPoints = np.linspace(1,N*N,N)

    for i in range(len(radius)):
        theta = np.linspace(0,2*np.pi,numPoints[i])
        for t in theta:
            x = np.cos(t) * radius[i]
            y = np.sin(t) * radius[i]
            grid.append(np.array([x,y]))
    return grid


def get_collision_time(av1Xo, av1Vo, av2Xo, av2Vo, Pt):
    D = av2Xo - av1Xo
    VoD = Pt - av1Xo
    VoDD = 2*VoD-av1Vo-av2Vo
    theta = get_angle(D,VoDD)
    # if theta < 0.001:
    #     theta = 0.0
    phi = np.arcsin(check_arcs(norm(D)*np.sin(theta)/(2.0*collisionRadius)))
    psi = np.pi - theta - phi
    # pdb.set_trace()
    FuturePt2 = []
    if phi != 0:
        FuturePt2 = D/norm(D) * get_rotation_matrix(theta) * (norm(D)*np.sin(psi)/np.sin(phi))
        FuturePt2 = np.squeeze(np.asarray(FuturePt2))
    else:
        FuturePt2 = D/norm(D) * (norm(D)-2.0*collisionRadius)
    FuturePt2 = FuturePt2 + av1Xo

    t21 = 0
    t22 = 0
    allt2 = (FuturePt2-av1Xo)/VoDD
    t2 = np.average(allt2)
    t21 = allt2[0]
    t22 = allt2[1]


    if t21 == 0 and t22 == 0:
        pdb.set_trace()
    elif t21 == 0 or np.isnan(t21):
        t2 = t22
    elif t22 == 0 or np.isnan(t22):
        t2 = t21
    else:
        t2 = (t21+t22)/2.0

        #(FuturePt2-av1Xo)/norm(FuturePt2-av1Xo)

    # VoD = Pt - av1Xo
    # step = P.Ts
    # t = step
    # FuturePt = av1Xo+(2*VoD-av1Vo-av2Vo)*t
    # # FuturePt = av1Xo+(VoD-av2Vo)*t
    # while norm(FuturePt-av2Xo) > av2.rpz*2.0:
    #     t = t + step
    #     FuturePt = av1Xo+(2*VoD-av1Vo-av2Vo)*t
    #     # FuturePt = av1Xo+(VoD-av2Vo)*t
    #     if t > 30:
    #         # pdb.set_trace()
    #         return t
    # print(FuturePt)
    # print(FuturePt2)
    # print(theta,psi,phi)
    # print(t2,t21,t22)
    # # if t < 0:
    # pdb.set_trace()
    return t2

# def set_goal_vel(av1, inRangeAV, direction):
def set_goal_vel(req):
    print("VO service")
    # print(req)
    # direction = req.desDirection #np.squeeze(np.asarray(direction))
    av1PreferedVoD = np.array([req.desVelocity.x, req.desVelocity.y])
    av1Xo = np.array([req.currentPosition.x, req.currentPosition.y])
    av1Vo = np.array([req.currentVelocity.x, req.currentVelocity.y])

    timePenWeight = 50.0

    winVoD = 0
    allAdVelPt = []


    maxAccel = 200
    accGrid = get_admissible_velocities(maxAccel)

    Pt = av1Xo+av1PreferedVoD



    allPenalties = []
    winPenalty = np.inf
    winVoD = 0


    for point in accGrid:
        Pt = av1Xo + av1Vo + point
        # if norm(Pt-av1Xo) > av1.maxSpeed:
        #     continue
        penalty = norm((av1Xo+av1PreferedVoD)-Pt)
        for i in range(len(req.otherPositions)):
            av2Xo = np.array([req.otherPositions[i].x, req.otherPositions[i].y])
            av2Vo = np.array([req.otherVelocities[i].x, req.otherVelocities[i].y])
            rvoTri = get_RVO_triangle(av1Xo, av1Vo, av2Xo, av2Vo)
            A = np.array(rvoTri[0])
            B = np.array(rvoTri[1])
            C = np.array(rvoTri[2])
            if barycentric(A,B,C,Pt):
                collisionTime = get_collision_time(av1Xo, av1Vo, av2Xo, av2Vo, Pt)
                # pdb.set_trace()
                penalty = penalty + timePenWeight / collisionTime
        if penalty < winPenalty:
            winPenalty = penalty
            winVoD = Pt
        allAdVelPt.append(Pt)
        allPenalties.append(penalty)
    # pdb.set_trace()
    av1VoD = winVoD-av1Xo

    # print(winVoD)
    return VOResponse([winVoD[0], winVoD[1]])



def VO_server():
    rospy.init_node('VO_server')
    s = rospy.Service('/VO', VO, set_goal_vel)
    print("VO Server Ready")
    rospy.spin()


if __name__ == "__main__":
    VO_server()
