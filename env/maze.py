from env.base_env import *
from env.map_data.maze_parser import *
import numpy as np

# A basic maze class that holds only walls,
# a start position and goal position
class BasicMazeEnv(BaseEnv):
    # The maze is 2 dimensional
    dim=2
    def __init__(self, 
            init_path='env/map_data/test_maze2.xml',
            xMax=300,
            yMax=300,
            ):
        super().__init__()
        self.xMax = xMax
        self.yMax = yMax
        # Load the XML file if it is valid and exists
        if init_path is not None and isValidMazeXML(init_path):
            self.startPos, self.endPos, self.walls = parseMazeXMLFile(init_path)
            self.obst = []
        else:
            # Otherwise we make everything empty
            self.start_pos, self.goal_pos, self.walls, self.obst = [], [], [], []

    # Return the positions of everything in the environment
    def getPositionData(self):
        return self.start_pos, self.goal_pos, self.walls, self.obst

    # Sample a random position within the map area
    def getRandomPosition(self):
        randX = np.random.uniform(low=-self.xMax, high=self.xMax)
        randY = np.random.uniform(low=-self.yMax, high=self.yMax)
        return [randX, randY]


