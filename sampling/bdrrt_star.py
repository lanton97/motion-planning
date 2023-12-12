import numpy as np
from sampling.graphs.base import *
from sampling.graphs.cost_graph import *
from collisionCheckers.base import PointCollisionChecker
from sampling.bdrrt import *
from sampling.costs.dist import distanceCost
# Base RRT planner developed from the description in
# http://lavalle.pl/rrt/about.html
# Should work in a variety of environments, as it is
# agnostic to vehicle dynamics, configuration space
# dimensionality, and collision checking
# Does assume only two types of obstacles: points
# and walls
class BidirectionalRRTStar(BidirectionalRRT):
    def __init__(self,
            environment,
            deltaConf,
            neighbourDist=20,
            positionCollisionChecker=PointCollisionChecker,
            vehicleDynamics=None,
            costFunction=distanceCost,
            ):
        super().__init__(environment, deltaConf, vehicleDynamics=vehicleDynamics, positionCollisionChecker=positionCollisionChecker)
        self.neighbourDist = neighbourDist
        self.costFunc = costFunction

    def plan(self, numSamples, render=True):
        initForwardNode = ConfigurationNode(self.initConfig)
        randOrient = self.dynamics.getRandomOrientation()
        endConfig = np.array([*self.env.endPos, *randOrient])
        initBackwardNode = ConfigurationNode(endConfig)
        forwardGraph = CostConfigurationGraph(len(self.initConfig), self.env.dim, initForwardNode)
        backwardGraph = CostConfigurationGraph(len(self.initConfig), self.env.dim, initBackwardNode)
        image_data = []

        for i in range(numSamples):
            self.expandGraph(forwardGraph)
            self.expandGraph(backwardGraph)

            if self.findConnectionAndConnect(forwardGraph, backwardGraph):
                break

            if render:
                image_data.append(self.render(forwardGraph, backwardGraph))

        if render:
            image_data.append(self.render(forwardGraph, backwardGraph))

        return forwardGraph, image_data

    # Expand a graph in place
    def expandGraph(self, graph):
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
        qNew, cost, connector = self.dynamics.sampleWCost(qNear, randConf, self.delConf, self.costFunc)
        # Update if there is a cheaper neighbour
        lowCostNeighbour = graph.getLowestCostNeighbour(qNew.config, self.neighbourDist, qNear, cost, self.costFunc)

        # Add the new node to the graph
        graph.addNode(lowCostNeighbour, qNew, self.costFunc, connector)

        # Rewire the tree based on the new node, if it is cheaper
        neighbours = graph.getNeighbourhoodNodes(qNew.config, self.neighbourDist)
        for neighbour in neighbours:
            graph.rewireIfCheaper(qNew, neighbour, self.costFunc, self.dynamics, self.neighbourDist)


    def findConnectionAndConnect(self, forwardGraph, backwardGraph):
        # We only need to check the most recent node of each graph of each node on the opposite graph to find a connection
        newestForwardNode = forwardGraph.nodes[-1]
        newestBackwardNode = backwardGraph.nodes[-1]
        nearestBWNodeToFW = backwardGraph.getNearestNode(newestForwardNode.config)
        nearestFWNodeToBW = forwardGraph.getNearestNode(newestBackwardNode.config)

        fwToBWDist = np.linalg.norm(newestForwardNode.config - nearestBWNodeToFW.config)
        bwToFWDist = np.linalg.norm(newestBackwardNode.config - nearestFWNodeToBW.config)
 
        connectionFound = False
        # TODO: Refactor the next part to reuse code
        if fwToBWDist < self.delConf:
            connectionFound = True
            newNode = nearestBWNodeToFW
            nearestNode = newestForwardNode
            while backwardGraph.nodes.index(newNode) != 0:
                forwardGraph.addNode(nearestNode, newNode, self.costFunc)
                nearestNode = newNode
                newNode = backwardGraph.edges[nearestNode]
            # Update once more to add the last root node
            forwardGraph.addNode(nearestNode, newNode, self.costFunc)


        if bwToFWDist < self.delConf:
            connectionFound = True
            newNode = newestBackwardNode
            nearestNode = nearestFWNodeToBW
            while backwardGraph.nodes.index(newNode) != 0:
                forwardGraph.addNode(nearestNode, newNode, self.costFunc)
                nearestNode = newNode
                newNode = backwardGraph.edges[nearestNode]
            # Update once more to add the last root node
            forwardGraph.addNode(nearestNode, newNode, self.costFunc)

        return connectionFound



