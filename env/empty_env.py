from env.base_env import BaseEnv
import numpy as np

# A basic maze class that holds only walls, a vehicle,
# a start position and goal position
class Empty2DEnv(BaseEnv):
    # Dimensionality is 2D
    dim=2
    def __init__(self, 
            xMax=300,
            yMax=300,
            startPos=[-250,10],
            endPos= [200, -150]
            ):
        super().__init__()
        # the maximum extends to the absolute, i.e. -300, 300
        self.xMax = xMax
        self.yMax = yMax
        self.startPos = startPos
        self.endPos = endPos
        # No walls or obstacles
        self.walls = []
        self.obst = []

    def getPositionData(self):
        return self.startPos, self.endPos, self.walls, self.obst

    def getRandomPosition(self):
        randX = np.random.uniform(low=-self.xMax, high=self.xMax)
        randY = np.random.uniform(low=-self.yMax, high=self.yMax)
        return [randX, randY]



