import numpy as np
from common.intersection import *
from common.geom import *

# A collision checking class that suports checking line-line intersections and
# Line-Arc intersections.
class CollChecker():
    def __init__(self,
            positionDimensionality,
            ):
        self.posDim = positionDimensionality

    # Check for collisions with the given edge and all of the obstacles in the environment.
    # Returns true if a collision is detected
    def checkCollisions(self, edge, env):
        walls = env.walls
        objects = env.obst
        xMax = env.xMax
        yMax = env.yMax
        collide = self.checkObjectCollisions(edge, objects) or self.checkWallCollisions(edge, walls) or \
                  self.checkInBounds(edge, xMax, yMax)


        return collide

    # We assume all objects consist of a number lines for this function
    def checkObjectCollisions(self, edge, objects):
        # Loop through the objects and their edges, then check if they intersect with any edges
        for obj in objects:
            for line in obj.edges:
                # Loop through the lines and arcs in the edge
                for connector in edge.connectors:
                    # Call the relevant intersection function
                    if type(connector) is arcSeg and checkArcLineIntersection(connector, line):
                        return True
                    elif type(connector) is straightLine and checkLineLineIntersection(connector, line):
                        return True
        return False

    # Walls are defined as lines
    def checkWallCollisions(self, edge, walls):
        # Loop through the wall lines, then check if they intersect with any edges
        for wall in walls:
            # Call the relevant collision functions for the edge component
            for connector in edge:
                if type(connector) is arcSeg and self.checkArcLineIntersection(connector, wall):
                    return True
                elif type(connector) is straightLine and self.checkLineLineIntersection(connector, wall):
                    return True
        return False

    # Helper function to find the line intersection
    def checkLineLineIntersection(self, line1, line2):
        return doLinesIntersect(line1.p1, line1.p2, line2.p1, line2.p2)

    # Helper function to find the line-arc intersection
    def checkArcLineIntersection(self, arc, line):
        return doArcLineIntersect(arc, line) 

    def checkInBounds(self, edge, xMax, yMax):
        for connector in edge:
            if type(connector) is arcSeg and checkCircleInBound(connector, xMax, yMax):
                return False
            elif type(connector) is straightLine and checkLineInBound(connector, xMax, yMax):
                return False
        return True



