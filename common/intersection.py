import numpy as np
from common.geom import *
from common.util import *
# Code adapted from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
# and https://www.mathworks.com/matlabcentral/answers/401724-how-to-check-if-a-line-segment-intersects-a-circle
# Given three collinear points p, q, r, the function checks if 
# point q lies on line segment 'pr' 

def onSegment(p, q, r): 
	if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
		(q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))): 
		return True
	return False

def orientation(p, q, r): 
	# to find the orientation of an ordered triplet (p,q,r) 
	# function returns the following values: 
	# 0 : Collinear points 
	# 1 : Clockwise points 
	# 2 : Counterclockwise 
	
	# See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
	# for details of below formula. 
	
	val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1])) 
	if (val > 0): 
		
		# Clockwise orientation 
		return 1
	elif (val < 0): 
		
		# Counterclockwise orientation 
		return 2
	else: 
		
		# Collinear orientation 
		return 0

# The main function that returns true if 
# the line segment 'p1q1' and 'p2q2' intersect. 
def doLinesIntersect(p1,q1,p2,q2): 
	
    # Find the 4 orientations required for 
    # the general and special cases 
    o1 = orientation(p1, q1, p2) 
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1) 

    # General case 
    if ((o1 != o2) and (o3 != o4)): 
	    return True

    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1 
    if (o1 == 0) and onSegment(p1, p2, q1):
        return True

    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1 
    if ((o2 == 0) and onSegment(p1, q2, q1)): 
        return True

    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2 
    if ((o3 == 0) and onSegment(p2, p1, q2)): 
        return True

    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2 
    if ((o4 == 0) and onSegment(p2, q1, q2)): 
        return True

    # If none of the cases 
    return False

def doArcLineIntersect(arc, line):
    dx = line.p2[0] - line.p1[0]
    dy = line.p2[1] - line.p1[1]
    d  = np.array([dx, dy])
    fx = line.p1[0] -arc.centre[0] 
    fy = line.p1[1] -arc.centre[1]
    f  = np.array([fx, fy])

    a = np.dot(d, d)
    b = 2*np.dot(f,d);
    c = np.dot(f,f) - arc.rad**2;
    discriminant = b*b-4*a*c;
    # The line doesnt intersect the arc at all
    if discriminant < 0:
        return False
    discriminant = np.sqrt( discriminant )
    t1 = (-b - discriminant)/(2*a);
    t2 = (-b + discriminant)/(2*a);
    intersect = False

    if( t1 >= 0 and t1 <= 1 ):
        p1x = dx * t1 + line.p1[0]
        p1y = dy * t2 + line.p1[1]
        # Check if the new point is on the line segment
        intersect = isOnLineAndArc([p1x, p1y], line, arc)

    if(t2 >= 0 and t2 <= 1 ) :
        p1x = dx * t2 + line.p1[0]
        p1y = dy * t2 + line.p1[1]
        intersect = intersect or isOnLineAndArc([p1x, p1y], line, arc)

    return intersect 

def isOnLineAndArc(point, line, arc):
    isOnLine = (line.p1[0] <= point[0] <= line.p2[0] and line.p1[1] <= point[1] <= line.p2[1] ) 
    isOnLine = isOnLine or  (line.p1[0] >= point[0] >= line.p2[0] and line.p1[1] >= point[1] >= line.p2[1] )
    pointAngle = np.arctan2(point[1] - arc.centre[1], point[0] - arc.centre[0]) 
    isOnArc = arc.th1 > pointAngle > arc.th2
    #TODO: Fix the arc thecking
    return isOnLine# and isOnArc

