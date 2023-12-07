import numpy as np
from sampling.graphs.base import *
from collisionCheckers.base import PointCollisionChecker
from sampling.base import *
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
            vehicleDynamics=None,
            ):
        super().__init__(initConfig, environment)
        self.delConf = deltaConf
        self.collChecker = positionCollisionChecker(2, 2, 2)
        self.dynamics = vehicleDynamics()

    def plan(self, numSamples, render=True):
        initNode = ConfigurationNode(self.initConfig)
        graph = ConfigurationGraph(len(self.initConfig), self.env.dim, initNode)
        image_data = []

        for i in range(numSamples):
            # Sample a random position
            randPos = self.env.getRandomPosition()
            # Check that the position is not a collision
            # This may need to change for more complex collisions than what is
            # implemented
            while self.collChecker.checkPointCollisions(randPos, self.env.obst) or \
                    self.collChecker.checkWallCollisions(randPos, self.env.walls):
                        randPos = self.env.getRandomPosition()

            randOrient = self.dynamics.getRandomOrientation()
            randConf = np.array([*randPos, *randOrient])
            # Get the nearest node to the position
            qNear = graph.getNearestNode(randPos)
            # Create a new configuration node closer to the random one by sampling
            # the vehicle dynamics
            # TODO: Add collision checking for new node
            qNew = self.dynamics.sample(qNear, randConf, self.delConf)


            # Add the new node to the graph
            graph.addNode(qNear, qNew)
            if self.collChecker.checkPointCollisions(self.env.endPos, [qNew.config], self.delConf):
                endNode = ConfigurationNode(self.env.endPos)
                graph.addNode(qNew, endNode)
                print("Path Found")
                break

            if render:
                image_data.append(self.render(graph))

        if render:
            image_data.append(self.render(graph))

        return graph, image_data





