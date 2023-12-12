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
class BidirectionalRRT(BaseSamplingPlanner):
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
        initForwardNode = ConfigurationNode(self.initConfig)
        initBackwardNode = ConfigurationNode(self.env.endPos)
        forwardGraph = ConfigurationGraph(len(self.initConfig), self.env.dim, initForwardNode)
        backwardGraph = ConfigurationGraph(len(self.initConfig), self.env.dim, initBackwardNode)
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
        qNew = self.dynamics.sample(qNear, randConf, self.delConf)

        # Add the new node to the graph
        graph.addNode(qNear, qNew)

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
                forwardGraph.addNode(nearestNode, newNode)
                nearestNode = newNode
                newNode = backwardGraph.edges[nearestNode]
            # Update once more to add the last root node
            forwardGraph.addNode(nearestNode, newNode)


        if bwToFWDist < self.delConf:
            connectionFound = True
            newNode = newestBackwardNode
            nearestNode = nearestFWNodeToBW
            while backwardGraph.nodes.index(newNode) != 0:
                forwardGraph.addNode(nearestNode, newNode)
                nearestNode = newNode
                newNode = backwardGraph.edges[nearestNode]
            # Update once more to add the last root node
            forwardGraph.addNode(nearestNode, newNode)

        return connectionFound



    def render(self, forwardGraph, backwardGraph):
        self.renderer.clear()
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = forwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines(edgeList)
        nodeList, edgeList = backwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        self.renderer.update()
        img_data = self.renderer.pull_array()
        return img_data


