from env.base_env import BaseEnv
import numpy as np

# An empty environment that contains a start and end position
# Used for the simplest planning scenario
class Empty2DEnv(BaseEnv):
    # Dimensionality is 2D
    dim=2
    def __init__(self, 
            xMax=300,
            yMax=300,
            startPos=[-250,10],
            endPos= [100, -130],
            init_path='env/map_data/test_maze2.xml',
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

    # Return the environment data for all of the important components
    def getPositionData(self):
        return self.startPos, self.endPos, self.walls, self.obst

    # Sample a random position within the bound of the map
    def getRandomPosition(self):
        randX = np.random.uniform(low=-self.xMax, high=self.xMax)
        randY = np.random.uniform(low=-self.yMax, high=self.yMax)
        return [randX, randY]



