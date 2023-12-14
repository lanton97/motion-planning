import numpy as np
from sampling.graphs.base import *
from collisionCheckers.coll import CollChecker
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
            environment,
            deltaConf,
            collChecker=CollChecker,
            vehicleDynamics=None,
            ):
        super().__init__(environment, vehicleDynamics)
        self.delConf = deltaConf
        self.collChecker = collChecker(self.env.dim)
        self.configNodeType = ConfigurationNode
        self.configGraphType = ConfigurationGraph

    def plan(self, numSamples, render=True):
        initNode = self.configNodeType(self.initConfig)
        graph = self.configGraphType(len(self.initConfig), self.env.dim, initNode)
        image_data = []

        for i in range(numSamples):
            # Sample a random position
            self.expandGraph(graph)

            if self.findConnectionAndConnect(graph):
                print("Path Found")
                break

            if render:
                image_data.append(self.render(graph))

        if render:
            image_data.append(self.render(graph))
            image_data.append(self.highlightFinalPath(graph))

        return graph, image_data

    def expandGraph(self, graph):
        # TODO: Fix sampling from free space in environment
        randPos = self.env.getRandomPosition()
        randOrient = self.dynamics.getRandomOrientation()
        randConf = np.array([*randPos, *randOrient])
        # Get the nearest node to the position
        qNear = graph.getNearestNode(randPos)
        # Create a new configuration node closer to the random one by sampling
        # the vehicle dynamics
        # TODO: Add collision checking for new node
        qNew, connector = self.dynamics.sample(qNear, randConf, self.delConf)

        # Check new connection for collisions
        if not self.collChecker.checkCollisions(connector, self.env):
            # Add the new node to the graph
            graph.addNode(qNear, qNew, connector)

    def findConnectionAndConnect(self, graph):
        mostRecentNode = graph.nodes[-1]
        connectionFound = False
        if np.linalg.norm(mostRecentNode.config[:self.env.dim] - self.env.endPos) < self.delConf:
            randOrient = self.dynamics.getRandomOrientation()
            endConf = np.array([*self.env.endPos, *randOrient])
            qNew, connector = self.dynamics.sample(mostRecentNode, endConf, self.delConf)

            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True
                graph.addNode(mostRecentNode, qNew, connector)

        return connectionFound




