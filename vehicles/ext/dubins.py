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

import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import math
import numpy as np
from enum import Enum

def getTurnCentres(start, target, turnRad):
    lstart_th = start[2] + np.pi/2
    rstart_th = start[2] - np.pi/2
    lend_th   = target[2] + np.pi/2
    rend_th   = target[2] - np.pi/2

    # Calculate the possible turn centres for the start and end turns
    lstart = np.array([start[0] + np.cos(lstart_th)*turnRad, start[1] + np.sin(lstart_th)*turnRad])
    rstart = np.array([start[0] + np.cos(rstart_th)*turnRad, start[1] + np.sin(rstart_th)*turnRad])
    lend = np.array([target[0] + np.cos(lend_th)*turnRad, target[1] + np.sin(lend_th)*turnRad])
    rend = np.array([target[0] + np.cos(rend_th)*turnRad, target[1] + np.sin(rend_th)*turnRad])

    return lstart, rstart, lend, rend

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

def rad2Deg(radians):
    return radians*180/np.pi

def deg2Rad(degrees):
    return degrees/180*np.pi

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

    q = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    x3 = (x1 + x2) / 2
    y3 = (y1 + y2) / 2

    xx = (r ** 2 - (q / 2) ** 2) ** 0.5 * (y1 - y2) / q
    yy = (r ** 2 - (q / 2) ** 2) ** 0.5 * (x2 - x1) / q
    return ((x3 + xx, y3 + yy), (x3 - xx, y3 - yy))

def getTheta(centre, point):
    x, y = [point[0]-centre[0], point[1]-centre[1]]
    theta = np.arctan2(y, x)
    return theta


def main():
    # User's waypoints: [x, y, heading (degrees)]
    pt1 = Waypoint(0,0,0)
    pt2 = Waypoint(80,70,260)
    #pt3 = Waypoint(1000,15000,180)
    #pt4 = Waypoint(0,0,270)
    Wptz = [pt1, pt2]
    # Run the code
    i = 0
    fig, ax = plt.subplots()

    while i<len(Wptz)-1:
        param = calcDubinsPath(Wptz[i], Wptz[i+1], 5)
        path, seg1_end, seg2_end, seg3_end = dubins_traj(param,1)


        # Plot the results
        #ax.plot(Wptz[i].x,Wptz[i].y,'kx')
        #ax.plot(Wptz[i+1].x,Wptz[i+1].y,'kx')
        #ax.plot(path[:,0],path[:,1],'b-')
        i+=1

    # Calculate possible circles
    '''q = sqrt((x2-x1)^2 + (y2-y1)^2)

    y3 = (y1+y2)/2

    x3 = (x1+x2)/2 

    x = x3 + sqrt(r^2-(q/2)^2)*(y1-y2)/q

    y = y3 + sqrt(r^2-(q/2)^2)*(x2-x1)/q  

    x = x3 - sqrt(r^2-(q/2)^2)*(y1-y2)/q

    y = y3 - sqrt(r^2-(q/2)^2)*(x2-x1)/q  '''
    
    begin_centres = centers([0,0, 0], seg1_end, param.turn_radius)
    centres = centers(seg2_end, seg3_end, param.turn_radius)

    th1 = getTheta(centres[0], seg2_end)
    th2 = getTheta(centres[0], seg3_end)

    #patch1 = Arc(centres[0], 10.0, 10.0, theta1=(seg2_end[2] + np.pi/2)*180/np.pi, theta2=(seg3_end[2] + np.pi/2)*180/np.pi, color='red', lw=1)
    patch1 = Arc(centres[0], 10.0, 10.0, theta1=(th1)*180/np.pi, theta2=(th2)*180/np.pi, color='red', lw=1)

    th2 = getTheta(begin_centres[1], path[0])
    th1 = getTheta(begin_centres[1], seg1_end)
    patch2 = Arc(begin_centres[1], 10.0, 10.0, theta1=(th1)*180/np.pi, theta2=(th2)*180/np.pi, color='red', lw=1)



    #ax.plot([0, seg1_end[0]], [0, seg1_end[1]]) 
    ax.plot([seg1_end[0], seg2_end[0]], [seg1_end[1], seg2_end[1]], 'r-') 
    #ax.plot([seg2_end[0], seg3_end[0]], [seg2_end[1], seg3_end[1]], 'r-') 

    ax.add_patch(patch1)
    ax.add_patch(patch2)

    #plt.grid(True)
    #plt.axis("equal")
    #plt.title('Dubin\'s Curves Trajectory Generation')
    #plt.xlabel('X')
    #plt.ylabel('Y')
    plt.show()


if __name__ == '__main__':
    main()