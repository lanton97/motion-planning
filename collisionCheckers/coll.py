import numpy as np
from common.intersection import *
from common.geom import *

class CollChecker():
    def __init__(self,
            positionDimensionality,
            ):
        self.posDim = positionDimensionality

    def checkCollisions(self, edge, env):
        walls = env.walls
        objects = env.obst
        collide = self.checkObjectCollisions(edge, objects) or self.checkWallCollisions(edge, walls)

        return collide

    # We assume all objects consist of lines
    def checkObjectCollisions(self, edge, objects):
        # Loop through the objects and their edges, then check if they intersect with any edges
        for obj in objects:
            for line in obj.edges:
                for connector in edge.connectors:
                    if type(connector) is arcSeg and checkArcLineIntersection(connector, line):
                        return True
                    elif type(connector) is straightLine and checkLineLineIntersection(connector, line):
                        return True
        return False

    def checkWallCollisions(self, edge, walls):
        # Loop through the objects and their edges, then check if they intersect with any edges
        for wall in walls:
            for connector in edge.connectors:
                if type(connector) is arcSeg and checkArcLineIntersection(connector, wall):
                    return True
                elif type(connector) is straightLine and checkLineLineIntersection(connector, wall):
                    return True
        return False

    def checkLineLineIntersection(self, line1, line2):
        return doLinesIntersect(line1.p1, line1.p2, line2.p1, line2.p2)

    def checkArcLineIntersection(self, arc, line):
        return doArcLineIntersect(arc, line) 



