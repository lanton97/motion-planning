import numpy as np
from sampling.graphs.cost_graph import *
from collisionCheckers.base import PointCollisionChecker
from sampling.rrt import *
from sampling.costs.dist import distanceCost
# Base RRT planner developed from the description in
# http://lavalle.pl/rrt/about.html
# Should work in a variety of environments, as it is
# agnostic to vehicle dynamics, configuration space
# dimensionality, and collision checking
# Does assume only two types of obstacles: points
# and walls
class RRTStar(RRT):
    def __init__(self,
            environment,
            deltaConf,
            neighbourDist=20,
            positionCollisionChecker=PointCollisionChecker,
            vehicleDynamics=None,
            costFunction=distanceCost,
            ):
        super().__init__(environment, deltaConf, positionCollisionChecker=positionCollisionChecker, vehicleDynamics=vehicleDynamics)
        self.neighbourDist=neighbourDist
        self.costFunc = costFunction
        self.configNodeType = ConfigurationNode
        self.configGraphType = CostConfigurationGraph

    def expandGraph(self, graph):
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
        qNew, cost, connector = self.dynamics.sampleWCost(qNear, randConf, self.delConf, self.costFunc)
        # Update if there is a cheaper neighbour
        lowCostNeighbour = graph.getLowestCostNeighbour(qNew.config, self.neighbourDist, qNear, cost, self.costFunc)

        # Add the new node to the graph
        graph.addNode(lowCostNeighbour, qNew, self.costFunc, connector)

        # Rewire the tree based on the new node, if it is cheaper
        neighbours = graph.getNeighbourhoodNodes(qNew.config, self.neighbourDist)
        for neighbour in neighbours:
            graph.rewireIfCheaper(qNew, neighbour, self.costFunc, self.dynamics, self.neighbourDist)


    def findConnectionAndConnect(self, graph):
        mostRecentNode = graph.nodes[-1]
        connectionFound = False
        if np.linalg.norm(mostRecentNode.config[:self.env.dim] - self.env.endPos) < self.delConf:
            connectionFound = True
            randOrient = self.dynamics.getRandomOrientation()
            endConf = np.array([*self.env.endPos, *randOrient])
            qNew, connector = self.dynamics.sampleWCost(mostRecentNode, endConf, self.delConf, self.costFunc)
            graph.addNode(mostRecentNode, qNew, self.costFunc, connector)

        return connectionFound
