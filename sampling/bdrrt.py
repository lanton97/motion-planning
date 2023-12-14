import numpy as np
from sampling.graphs.base import *
from collisionCheckers.coll import CollChecker
from sampling.base import *
from viz.viz import *

# A class for the bidirectional RRT algorithm
class BidirectionalRRT(BaseSamplingPlanner):
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

    # This function runs the loop for sampling and expanding two trees until a path is found
    # Similar to the rrt function but uses two trees
    def plan(self, numSamples, render=True):
        # Initialize the forward and backward graphs
        initForwardNode = self.configNodeType(self.initConfig)
        randOrient = self.dynamics.getRandomOrientation()
        endConfig = np.array([*self.env.endPos, *randOrient])
        initBackwardNode = self.configNodeType(endConfig)
        forwardGraph = self.configGraphType(len(self.initConfig), self.env.dim, initForwardNode)
        backwardGraph = self.configGraphType(len(self.initConfig), self.env.dim, initBackwardNode)
        image_data = []

        # Sample for a maximum of N nodes
        for i in range(numSamples):
            # Exapnd the forward and backward node once
            self.expandGraph(forwardGraph)
            self.expandGraph(backwardGraph)

            # Check if we can connect the forward and backward graphs and break if we can
            if self.findConnectionAndConnect(forwardGraph, backwardGraph):
                print('Path Found.')
                break

            if render:
                image_data.append(self.render(forwardGraph, backwardGraph))

        if render:
            image_data.append(self.render(forwardGraph, backwardGraph))
            image_data.append(self.highlightFinalPath(forwardGraph, backwardGraph))

        return forwardGraph, image_data

    # Expand a graph in place
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

    # Check if we can connect the two trees and do so if possibe
    # Return true if we make a connection and false otherwise
    def findConnectionAndConnect(self, forwardGraph, backwardGraph):
        # We only need to check the most recent node of each graph of each node on the opposite graph to find a connection
        newestForwardNode = forwardGraph.nodes[-1]
        newestBackwardNode = backwardGraph.nodes[-1]
        nearestBWNodeToFW = backwardGraph.getNearestNode(newestForwardNode.config)
        nearestFWNodeToBW = forwardGraph.getNearestNode(newestBackwardNode.config)

        # Get the distance of the new nodes to the closest nodes of the opposite graph
        fwToBWDist = np.linalg.norm(newestForwardNode.config - nearestBWNodeToFW.config)
        bwToFWDist = np.linalg.norm(newestBackwardNode.config - nearestFWNodeToBW.config)
 
        connectionFound = False
        # TODO: Refactor the next part to reuse code
        # If we can connect, try to
        if fwToBWDist < self.delConf:
            newNode = nearestBWNodeToFW
            nearestNode = newestForwardNode
            # Get a new motion between the nodes
            _, connector = self.dynamics.sample(nearestNode, newNode.config, self.delConf)
            # Check if the path is through free space
            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True

                # Loop through the backward graph to add all of the nodes to the forward graph
                while backwardGraph.nodes.index(newNode) != 0:
                    forwardGraph.addNode(nearestNode, newNode, connector)
                    nearestNode = newNode
                    newNode = backwardGraph.edges[nearestNode].parentNode
                    connector = backwardGraph.edges[nearestNode].connectors
                # Update once more to add the last root node
                forwardGraph.addNode(nearestNode, newNode, connector)


        # If we can connect, try to
        if bwToFWDist < self.delConf:
            newNode = newestBackwardNode
            nearestNode = nearestFWNodeToBW
            # Get a new motion between the nodes
            _, connector = self.dynamics.sample(nearestNode, newNode.config, self.delConf)
            # Check if the path is through free space
            if not self.collChecker.checkCollisions(connector, self.env):
                connectionFound = True

                # Loop through the backward graph to add all of the nodes to the forward graph
                while backwardGraph.nodes.index(newNode) != 0:
                    forwardGraph.addNode(nearestNode, newNode, connector)
                    nearestNode = newNode
                    newNode = backwardGraph.edges[nearestNode].parentNode
                    connector = backwardGraph.edges[nearestNode].connectors
                # Update once more to add the last root node
                forwardGraph.addNode(nearestNode, newNode, connector)

        return connectionFound

    # We update the render functions from the base to support the two graphs
    def render(self, forwardGraph, backwardGraph):
        self.renderer.clear()
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = forwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        nodeList, edgeList = backwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        self.renderer.update()
        img_data = self.renderer.pull_array()
        return img_data

    def highlightFinalPath(self, forwardGraph, backwardGraph):
        self.renderer.clear()
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = forwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        nodeList, edgeList = backwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        _, finalEdges = forwardGraph.getPath()
        self.renderer.draw_lines_and_curves(finalEdges, colour=RED)
        self.renderer.update()
        img_data = self.renderer.pull_array()
        return img_data

