import numpy as np
# Collision checker for sampling new configurations
# We treat each configuration as a sphere/circle
# of a given radius to simplify
# New collision checkers can easily be implemented by extending this class
# and overloading the functions
class PointCollisionChecker():
    def __init__(self,
            positionDimensionality,
            configurationDimensionality,
            collDist,
            ):
        self.posDim = positionDimensionality
        self.confDim = configurationDimensionality
        self.collDist = collDist

    def checkPointCollisions(self, config, pointList):
        pos = np.array(config[:self.posDim])
        for point in pointList:
            dist = np.linalg.norm(pos - np.array(point))
            if dist <= self.collDist:
                return True
        return False

    def checkWallCollisions(self, config, wallList):
        pos = np.array(config[:self.posDim])
        for wall in wallList:
            dist = np.linalg.norm(pos - np.array(point))
            if dist <= self.collDist:
                return True
        return False





