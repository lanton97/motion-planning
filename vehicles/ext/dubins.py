"""
Calculate Dubins Curve between waypoints

author: Fischer, but I just copied the math from this paper:

fischergabbert@gmail.com

http://mems.eng.uci.edu/files/2014/04/Dubins_Set_Robotics_2001.pdf

Andrew Walker did this in C and I used that as a reference too..his github is out there somewhere

"""

"""
TODOS:

- Reduce computation time using the classification methods in the paper

"""
# NOTE: This is a modified version of the original code, mainly changed to
# support output in radians and as a series of straight segments and arcs
# This is useful for plotting later on

import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import math
import numpy as np
from enum import Enum
from common.geom import straightLine, arcSeg
from common.util import rad2Deg, deg2Rad


class TurnType(Enum):
    LSL = 1
    LSR = 2
    RSL = 3
    RSR = 4
    RLR = 5
    LRL = 6

class Waypoint:

    def __init__(self, x, y, psi):
        self.x = x
        self.y = y
        self.psi = psi

    def __str__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + ", psi: " + str(self.psi)

class Param:
    def __init__(self, p_init, seg_final, turn_radius,):
        self.p_init = p_init
        self.seg_final = seg_final
        self.turn_radius = turn_radius
        self.type = 0

class Trajectory:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def wrapTo360(angle):
    posIn = angle>0
    angle = angle % 360
    if angle == 0 and posIn:
        angle = 360
    return angle

def wrapTo180(angle):
    q = (angle < -180) or (180 < angle)
    if(q):
        angle = wrapTo360(angle + 180) - 180
    return angle

def headingToStandard(hdg):
    # Convert NED heading to standard unit cirlce...degrees only for now (Im lazy)
    thet = wrapTo360(90 - wrapTo180(hdg))
    return thet

def calcDubinsPath(wpt1, wpt2, turn_radius):
    # Calculate a dubins path between two waypoints
    param = Param(wpt1, 0, 0)
    tz        = [0, 0, 0, 0, 0, 0]
    pz        = [0, 0, 0, 0, 0, 0]
    qz        = [0, 0, 0, 0, 0, 0]
    param.seg_final = [0, 0, 0]
    # Convert the headings from NED to standard unit cirlce, and then to radians
    psi1 = headingToStandard(wpt1.psi)*math.pi/180
    psi2 = headingToStandard(wpt2.psi)*math.pi/180

    # Do math
    param.turn_radius = turn_radius# (vel*vel)/(9.8*math.tan(phi_lim*math.pi/180))
    dx = wpt2.x - wpt1.x
    dy = wpt2.y - wpt1.y
    D = math.sqrt(dx*dx + dy*dy)
    d = D/param.turn_radius # Normalize by turn radius...makes length calculation easier down the road.

    # Angles defined in the paper
    theta = math.atan2(dy,dx) % (2*math.pi)
    alpha = (psi1 - theta) % (2*math.pi)
    beta  = (psi2 - theta) % (2*math.pi)
    best_word = -1
    best_cost = -1

    # Calculate all dubin's paths between points
    tz[0], pz[0], qz[0] = dubinsLSL(alpha,beta,d)
    tz[1], pz[1], qz[1] = dubinsLSR(alpha,beta,d)
    tz[2], pz[2], qz[2] = dubinsRSL(alpha,beta,d)
    tz[3], pz[3], qz[3] = dubinsRSR(alpha,beta,d)
    tz[4], pz[4], qz[4] = dubinsRLR(alpha,beta,d)
    tz[5], pz[5], qz[5] = dubinsLRL(alpha,beta,d)

    # Now, pick the one with the lowest cost
    for x in range(6):
        if(tz[x]!=-1):
            cost = tz[x] + pz[x] + qz[x]
            if(cost<best_cost or best_cost==-1):
                best_word = x+1
                best_cost = cost
                param.seg_final = [tz[x],pz[x],qz[x]]
                #param.seg_final = [tz[x]*param.turn_radius,pz[x]*param.turn_radius,qz[x]*param.turn_radius]

    param.type = TurnType(best_word)
    return param

# Here's all of the dubins path math
def dubinsLSL(alpha, beta, d):
    tmp0      = d + math.sin(alpha) - math.sin(beta)
    tmp1      = math.atan2((math.cos(beta)-math.cos(alpha)),tmp0)
    p_squared = 2 + d*d - (2*math.cos(alpha-beta)) + (2*d*(math.sin(alpha)-math.sin(beta)))
    if p_squared<0:
        p=-1
        q=-1
        t=-1
    else:
        t         = (tmp1-alpha) % (2*math.pi)
        p         = math.sqrt(p_squared)
        q         = (beta - tmp1) % (2*math.pi)
    return t, p, q

def dubinsRSR(alpha, beta, d):
    tmp0      = d - math.sin(alpha) + math.sin(beta)
    tmp1      = math.atan2((math.cos(alpha)-math.cos(beta)),tmp0)
    p_squared = 2 + d*d - (2*math.cos(alpha-beta)) + 2*d*(math.sin(beta)-math.sin(alpha))
    if p_squared<0:
        p=-1
        q=-1
        t=-1
    else:
        t         = (alpha - tmp1 ) % (2*math.pi)
        p         = math.sqrt(p_squared)
        q         = (-1*beta + tmp1) % (2*math.pi)
    return t, p, q

def dubinsRSL(alpha,beta,d):
    tmp0      = d - math.sin(alpha) - math.sin(beta)
    p_squared = -2 + d*d + 2*math.cos(alpha-beta) - 2*d*(math.sin(alpha) + math.sin(beta))
    if p_squared<0:
        p=-1
        q=-1
        t=-1
    else:
        p         = math.sqrt(p_squared)
        tmp2      = math.atan2((math.cos(alpha)+math.cos(beta)),tmp0) - math.atan2(2,p)
        t         = (alpha - tmp2) % (2*math.pi)
        q         = (beta - tmp2) % (2*math.pi)
    return t, p, q

def dubinsLSR(alpha, beta, d):
    tmp0      = d + math.sin(alpha) + math.sin(beta)
    p_squared = -2 + d*d + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha) + math.sin(beta))
    if p_squared<0:
        p=-1
        q=-1
        t=-1
    else:
        p         = math.sqrt(p_squared)
        tmp2      = math.atan2((-1*math.cos(alpha)-math.cos(beta)),tmp0) - math.atan2(-2,p)
        t         = (tmp2 - alpha) % (2*math.pi)
        q         = (tmp2 - beta) % (2*math.pi)
    return t, p, q

def dubinsRLR(alpha, beta, d):
    tmp_rlr = (6 - d*d + 2*math.cos(alpha-beta) + 2*d*(math.sin(alpha)-math.sin(beta)))/8
    if(abs(tmp_rlr)>1):
        p=-1
        q=-1
        t=-1
    else:
        p = (2*math.pi - math.acos(tmp_rlr)) % (2*math.pi)
        t = (alpha - math.atan2((math.cos(alpha)-math.cos(beta)), d-math.sin(alpha)+math.sin(beta)) + p/2 % (2*math.pi)) % (2*math.pi)
        q = (alpha - beta - t + (p % (2*math.pi))) % (2*math.pi)

    return t, p, q

def dubinsLRL(alpha, beta, d):
    tmp_lrl = (6 - d*d + 2*math.cos(alpha-beta) + 2*d*(-1*math.sin(alpha)+math.sin(beta)))/8
    if(abs(tmp_lrl)>1):
        p=-1
        q=-1
        t=-1
    else:
        p = (2*math.pi - math.acos(tmp_lrl)) % (2*math.pi)
        t = (-1*alpha - math.atan2((math.cos(alpha)-math.cos(beta)), d+math.sin(alpha)-math.sin(beta)) + p/2) % (2*math.pi)
        q = ((beta % (2*math.pi))-alpha-t+(p % (2*math.pi))) % (2*math.pi)
    return t, p, q

def dubins_traj(param,step):
    # Build the trajectory from the lowest-cost path
    x = 0
    i = 0
    length = (param.seg_final[0]+param.seg_final[1]+param.seg_final[2])*param.turn_radius
    length = math.floor(length/step)
    path = -1 * np.ones((length,3))
    seg1_end = None
    seg2_end = None
    seg3_end = None
    param1 = param.seg_final[0]
    param2 = param.seg_final[1]

    while x < length:
        path[i] = dubins_path(param,x)
        tprime = x / param.turn_radius
        if(tprime<param1):
            seg1_end = path[i]
        elif(tprime<(param1+param2)):
            seg2_end = path[i]
        else:
            seg3_end = path[i]
        x += step
        i+=1

    return path, seg1_end, seg2_end, seg3_end


def dubins_path(param, t):
    # Helper function for curve generation
    tprime = t/param.turn_radius
    p_init = np.array([0,0,headingToStandard(param.p_init.psi)*math.pi/180])
    #
    L_SEG = 1
    S_SEG = 2
    R_SEG = 3
    DIRDATA = np.array([[L_SEG,S_SEG,L_SEG],[L_SEG,S_SEG,R_SEG],[R_SEG,S_SEG,L_SEG],[R_SEG,S_SEG,R_SEG],[R_SEG,L_SEG,R_SEG],[L_SEG,R_SEG,L_SEG]])
    #
    types = DIRDATA[param.type.value-1][:]
    param1 = param.seg_final[0]
    param2 = param.seg_final[1]
    mid_pt1 = dubins_segment(param1,p_init,types[0])
    mid_pt2 = dubins_segment(param2,mid_pt1,types[1])

    if(tprime<param1):
        end_pt = dubins_segment(tprime,p_init,types[0])
    elif(tprime<(param1+param2)):
        end_pt = dubins_segment(tprime-param1,mid_pt1,types[1])
    else:
        end_pt = dubins_segment(tprime-param1-param2, mid_pt2, types[2])

    end_pt[0] = end_pt[0] * param.turn_radius + param.p_init.x
    end_pt[1] = end_pt[1] * param.turn_radius + param.p_init.y
    end_pt[2] = end_pt[2] % (2*math.pi)

    return end_pt

def dubins_segment(seg_param, seg_init, seg_type):
    # Helper function for curve generation
    L_SEG = 1
    S_SEG = 2
    R_SEG = 3
    seg_end = np.array([0.0,0.0,0.0])
    if( seg_type == L_SEG ):
        seg_end[0] = seg_init[0] + math.sin(seg_init[2]+seg_param) - math.sin(seg_init[2])
        seg_end[1] = seg_init[1] - math.cos(seg_init[2]+seg_param) + math.cos(seg_init[2])
        seg_end[2] = seg_init[2] + seg_param
    elif( seg_type == R_SEG ):
        seg_end[0] = seg_init[0] - math.sin(seg_init[2]-seg_param) + math.sin(seg_init[2])
        seg_end[1] = seg_init[1] + math.cos(seg_init[2]-seg_param) - math.cos(seg_init[2])
        seg_end[2] = seg_init[2] - seg_param
    elif( seg_type == S_SEG ):
        seg_end[0] = seg_init[0] + math.cos(seg_init[2]) * seg_param
        seg_end[1] = seg_init[1] + math.sin(seg_init[2]) * seg_param
        seg_end[2] = seg_init[2]

    return seg_end

def centers(p1, p2, r):
    x1, y1, a1 = p1 
    x2, y2, a2 = p2 

    q = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 + 0.01 
    x3 = (x1 + x2) / 2
    y3 = (y1 + y2) / 2

    xx = np.sqrt(np.abs(r ** 2 - (q / 2) ** 2)) * (y1 - y2) / q
    yy = np.sqrt(np.abs(r ** 2 - (q / 2) ** 2)) * (x2 - x1) / q
    return np.array([(x3 + xx, y3 + yy), (x3 - xx, y3 - yy)])

def getTheta(centre, point):
    x, y = [point[0]-centre[0], point[1]-centre[1]]
    theta = np.arctan2(y, x)
    return theta

# Return an arc segment, straight line, and arcsegment for plotting
def getComponents(param, points):
    curveType = param.type
    # which centre is chosen and the order of the thetas for the
    # arc are dependent on the direction of the curve
    firstCurve = None
    if curveType in [TurnType.RSL, TurnType.RSR, TurnType.RLR]:
        _, centre = centers(points[0], points[1], param.turn_radius)
        th1 = getTheta(centre, points[1]) 
        th2 = getTheta(centre, points[0])
        firstCurve = arcSeg(centre, th1, th2, param.turn_radius)
    else:
        centre,_ = centers(points[0], points[1], param.turn_radius)
        th1 = getTheta(centre, points[0]) 
        th2 = getTheta(centre, points[1])
        firstCurve = arcSeg(centre, th1, th2, param.turn_radius)
    
    # Similar to before but account for the possibility of straight or curved segmentts
    segment2 = None
    if curveType in [TurnType.LSR, TurnType.RSR, TurnType.RSL, TurnType.LSL]:
        segment2 = straightLine(points[1], points[2])
    elif curveType in[TurnType.LRL, TurnType.RLR]:
        _, centre = centers(points[1], points[2], param.turn_radius)
        th1 = getTheta(centre, points[2]) 
        th2 = getTheta(centre, points[1])
        segment2 = arcSeg(centre, th1, th2, param.turn_radius)
    else:
        centre,_ = centers(points[1], points[2], param.turn_radius)
        th1 = getTheta(centre, points[1]) 
        th2 = getTheta(centre, points[2])
        segment2 = arcSeg(centre, th1, th2, param.turn_radius)

    secondCurve = None
    if curveType in [TurnType.LSR, TurnType.RSR]:
        _, centre = centers(points[2], points[3], param.turn_radius)
        th1 = getTheta(centre, points[3]) 
        th2 = getTheta(centre, points[2])
        secondCurve = arcSeg(centre, th1, th2, param.turn_radius)
    else:
        centre,_ = centers(points[2], points[3], param.turn_radius)
        th1 = getTheta(centre, points[2]) 
        th2 = getTheta(centre, points[3])
        secondCurve = arcSeg(centre, th1, th2, param.turn_radius)

    return firstCurve, segment2, secondCurve

# Find the shortest dubins path given a start and end configuration and constraints
def getClosedFormPath(start, target, turnRad):
    pt1 = Waypoint(start[0], start[1], rad2Deg(start[2]))
    pt2 = Waypoint(target[0], target[1], rad2Deg(target[2]))
    param = calcDubinsPath(pt1, pt2, turnRad)
    path, seg1_end, seg2_end, seg3_end = dubins_traj(param,1)

    points = [path[0], seg1_end, seg2_end, target]
    return getComponents(param, points)



def main():
    # User's waypoints: [x, y, heading (degrees)]
    fig, ax = plt.subplots()

    arc1, straight, arc2 =  getClosedFormPath((0,0,0), (-80,70,2.5), 5)
    patch1 = Arc(arc1.centre, 10.0, 10.0, theta1=(arc1.th1)*180/np.pi, theta2=(arc1.th2)*180/np.pi, color='red', lw=1)
    patch2 = Arc(arc2.centre, 10.0, 10.0, theta1=(arc2.th1)*180/np.pi, theta2=(arc2.th2)*180/np.pi, color='red', lw=1)
    ax.plot([straight.p1[0], straight.p2[0]], [straight.p1[1], straight.p2[1]], 'r-') 

    ax.add_patch(patch1)
    ax.add_patch(patch2)

    plt.grid(True)
    plt.axis("equal")
    plt.title('Dubin\'s Curves Trajectory Generation')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


if __name__ == '__main__':
    main()
