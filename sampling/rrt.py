import numpy as np
from .base import *
from collisionCheckers.base import PointCollisionChecker
# Base RRT planner developed from the description in
# http://lavalle.pl/rrt/about.html
# Should work in a variety of environments, as it is
# agnostic to vehicle dynamics, configuration space
# dimensionality, and collision checking
# Does assume only two types of obstacles: points
# and walls
class RRT(BaseSamplingPlanner):
    def __init__(self,
            initConfig,
            environment,
            deltaConf,
            positionCollisionChecker=PointCollisionChecker,
            vehicleDynamics=None
            ):
        self.initConfig = initConfig
        self.env = environment
        self.delConf = deltaConf
        self.collChecker = positionCollisionChecker
        self.dynamics = vehicleDynamics

    def plan(self, numSamples):
        initNode = ConfigurationNode(self.initConfig)
        graph = ConfigurationGraph(len(self.initConfig), self.env.dim, initNode)

        for i in range(numSamples):
            # Sample a random position
            randPos = self.env.getRandomPosition()
            # Check that the position is not a collision
            # This may need to change for more complex collisions than what is
            # implemented
            while PointCollisionChecker.checkPointCollisions(randPos, self.env.obst) or \
                    PointCollisionChecker.checkWallsCollisions(randPos, self.env.walls):
                        randPos = self.env.getRandomPosition()

            randOrient = vehicleDynamics.getRandomOrientation()
            randConf = np.array([*randPos, *randOrient])
            # Get the nearest node to the position
            qNear = graph.getNearestNode(randPos)
            # Create a new configuration node closer to the random one by sampling
            # the vehicle dynamics
            # TODO: Add collision checking for new node
            qNew = self.dynamics.sample(qNear, randConf, self.delConf)
            # Add the new node to the graph
            graph.addNode(qNear, qNew)

        return graph





