from sampling import *
from env import *
from vehicles import *

algorithms = {
        'rrt': RRT,
        'rrt*': RRTStar,
        'bd-rrt': BidirectionalRRT,
        'bd-rrt*': BidirectionalRRTStar,
        }

vehicles = {
        'particle-2d': particleDynamics2D,
        }

maps = {
        'empty-2D': Empty2DEnv,
        'maze-2D': BasicMazeEnv,
        }
