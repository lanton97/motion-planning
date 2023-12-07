from env.base_env import *
from env.map_data.maze_parser import *


# A basic maze class that holds only walls, a vehicle,
# a start position and goal position
class BasicMazeEnv(BaseEnv):
    def __init__(self, 
            init_path=None,
            ):
        super().__init__()
        if init_path is not None and isValidMazeXML(init_path):
            self.start_pos, self.goal_pos, self.walls = parseMazeXMLFile(init_path)
        else:
            self.start_pos, self.goal_pos, self.walls = [], [], []

    def getPositionData(self):
        return self.start_pos, self.goal_pos, self.walls, self.vehicle.get_pos()


