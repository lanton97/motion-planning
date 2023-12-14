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

    # This function runs the loop for sampling and expanding the tree until a path is found
    def plan(self, numSamples, render=True):
        # Initialize the grapgs
        initNode = self.configNodeType(self.initConfig)
        graph = self.configGraphType(len(self.initConfig), self.env.dim, initNode)
        image_data = []

        # Sample up to N times
        for i in range(numSamples):
            # expand the graph once
            self.expandGraph(graph)

            # Check if the graph has a connection to the end
            if self.findConnectionAndConnect(graph):
                # If so, exit
                print("Path Found")
                break

            if render:
                image_data.append(self.render(graph))

        if render:
            image_data.append(self.render(graph))
            image_data.append(self.highlightFinalPath(graph))

        return graph, image_data

    # Add a new node to the graph in-place
    def expandGraph(self, graph):
        # Sample a random configuration
        randPos = self.env.getRandomPosition()
        randOrient = self.dynamics.getRandomOrientation()
        randConf = np.array([*randPos, *randOrient])
        # Get the nearest node to the position
        qNear = graph.getNearestNode(randPos)
        # Create a new configuration node closer to the random one by sampling
        # the vehicle dynamics
        qNew, connector = self.dynamics.sample(qNear, randConf, self.delConf)

        # Check new connection for collisions
        if not self.collChecker.checkCollisions(connector, self.env):
            # Add the new node to the graph
            graph.addNode(qNear, qNew, connector)

    # Check if we can connect the graph to the final position, and do so
    # Return true if a connection has been made, false otherwise
    def findConnectionAndConnect(self, graph):
        # Get the most recent node, as this is the only one we haven't checkd yet
        mostRecentNode = graph.nodes[-1]
        connectionFound = False
        # Check if we are close enough to connect to the final node
        if np.linalg.norm(mostRecentNode.config[:self.env.dim] - self.env.endPos) < self.delConf:
            # Get a random orientation for the final node
            randOrient = self.dynamics.getRandomOrientation()
            endConf = np.array([*self.env.endPos, *randOrient])
            # Create a new connection and node using the vehicle dynamics
            qNew, connector = self.dynamics.sample(mostRecentNode, endConf, self.delConf)
            # Check for collisions before connecting
            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True
                graph.addNode(mostRecentNode, qNew, connector)

        return connectionFound




