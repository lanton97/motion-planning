import numpy as np
from sampling.graphs.cost_graph import *
from collisionCheckers.coll import CollChecker
from sampling.rrt import *
from sampling.costs.dist import distanceCost

# An improvement on RRT that uses a cost metric to select better connections
# and rewire nodes to better parents
class RRTStar(RRT):
    def __init__(self,
            environment,
            deltaConf,
            neighbourDist=20,
            collisionChecker=CollChecker,
            vehicleDynamics=None,
            costFunction=distanceCost,
            ):
        super().__init__(environment, deltaConf, collChecker=collisionChecker, vehicleDynamics=vehicleDynamics)
        self.neighbourDist=neighbourDist
        self.costFunc = costFunction
        self.configNodeType = ConfigurationNode
        self.configGraphType = CostConfigurationGraph

    # Add a new node to the grapg in place
    def expandGraph(self, graph):
        # Sample a random configuration
        randPos = self.env.getRandomPosition()
        randOrient = self.dynamics.getRandomOrientation()
        randConf = np.array([*randPos, *randOrient])
        # Get the nearest node to the position
        qNear = graph.getNearestNode(randPos)

        # Create a new configuration node closer to the random one by sampling
        # the vehicle dynamics
        qNew, cost, connector = self.dynamics.sampleWCost(qNear, randConf, self.delConf, self.costFunc)
        # Update the nearest node if there is a cheaper neighbour
        lowCostNeighbour = graph.getLowestCostNeighbour(qNew.config, self.neighbourDist, qNear, cost, self.costFunc)

        # Check new connection for collisions
        if not self.collChecker.checkCollisions(connector, self.env):
            # Add the new node to the graph
            graph.addNode(lowCostNeighbour, qNew, self.costFunc, connector)

            # Rewire the tree based on the new node, if it is cheaper
            neighbours = graph.getNeighbourhoodNodes(qNew.config, self.neighbourDist)
            for neighbour in neighbours:
                graph.rewireIfCheaper(qNew, neighbour, self.costFunc, self.dynamics, self.neighbourDist)

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
            qNew, cost, connector = self.dynamics.sampleWCost(mostRecentNode, endConf, self.delConf, self.costFunc)
            # Check for collisions before connecting
            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True
                graph.addNode(mostRecentNode, qNew, self.costFunc, connector)

        return connectionFound
