from env.base_env import *
from env.map_data.maze_parser import *
import numpy as np


# A basic maze class that holds only walls, a vehicle,
# a start position and goal position
class BasicMazeEnv(BaseEnv):
    dim=2
    def __init__(self, 
            init_path='env/map_data/test_maze2.xml',
            xMax=300,
            yMax=300,
            ):
        super().__init__()
        self.xMax = xMax
        self.yMax = yMax
        if init_path is not None and isValidMazeXML(init_path):
            self.startPos, self.endPos, self.walls = parseMazeXMLFile(init_path)
            self.obst = []
        else:
            self.start_pos, self.goal_pos, self.walls, self.obst = [], [], [], []

    def getPositionData(self):
        return self.start_pos, self.goal_pos, self.walls, self.obst

    def getRandomPosition(self):
        randX = np.random.uniform(low=-self.xMax, high=self.xMax)
        randY = np.random.uniform(low=-self.yMax, high=self.yMax)
        return [randX, randY]


