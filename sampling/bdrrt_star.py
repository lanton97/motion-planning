import numpy as np
from sampling.graphs.base import *
from sampling.graphs.cost_graph import *
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
            collChecker=CollChecker,
            vehicleDynamics=None,
            costFunction=distanceCost,
            ):
        super().__init__(environment, deltaConf, vehicleDynamics=vehicleDynamics, collChecker=collChecker)
        self.neighbourDist=neighbourDist
        self.costFunc = costFunction
        self.configNodeType = ConfigurationNode
        self.configGraphType = CostConfigurationGraph


    # Expand a graph in place
    def expandGraph(self, graph):
        # Sample a random position
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

        # Check new connection for collisions
        if not self.collChecker.checkCollisions(connector, self.env):
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

            newNode = nearestBWNodeToFW
            nearestNode = newestForwardNode
            _, cost, connector = self.dynamics.sampleWCost(nearestNode, newNode.config, self.delConf, self.costFunc)
            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True
                while backwardGraph.nodes.index(newNode) != 0:
                    forwardGraph.addNode(nearestNode, newNode, self.costFunc, connector)
                    nearestNode = newNode
                    newNode = backwardGraph.edges[nearestNode].parentNode
                    connector = backwardGraph.edges[nearestNode].connectors
                # Update once more to add the last root node
                forwardGraph.addNode(nearestNode, newNode, self.costFunc, connector)


        if bwToFWDist < self.delConf:
            newNode = newestBackwardNode
            nearestNode = nearestFWNodeToBW
            _, cost, connector = self.dynamics.sampleWCost(nearestNode, newNode.config, self.delConf, self.costFunc)
            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True
                while backwardGraph.nodes.index(newNode) != 0:
                    forwardGraph.addNode(nearestNode, newNode, self.costFunc, connector)
                    nearestNode = newNode
                    newNode = backwardGraph.edges[nearestNode].parentNode
                    connector = backwardGraph.edges[nearestNode].connectors
                # Update once more to add the last root node
                forwardGraph.addNode(nearestNode, newNode, self.costFunc, connector)

        return connectionFound



