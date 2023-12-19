from sampling import *
from env import *
from vehicles import *

algorithms = {
        'rrt': RRT,
        'rrt*': RRTStar,
        'bd-rrt': BidirectionalRRT,
        'bd-rrt*': BidirectionalRRTStar,
        'fmt': FMT,
        }

vehicles = {
        'particle-2d': particleDynamics2D,
        'dubins': dubinsCar
        }

maps = {
        'empty-2D': Empty2DEnv,
        'maze-2D': BasicMazeEnv,
        }


