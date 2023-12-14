import numpy as np
from copy import deepcopy
from common.geom import *

# A class for a node in the graph
class ConfigurationNode():
    def __init__(self, configuration):
        self.config = configuration

# This edge class holds the parent node and the connections
class Edge():
    def __init__(self, parentNode, connectors):
        self.connectors = connectors
        self.parentNode = parentNode

# This graph is used to hold the configurations and connections
# for sampling based planners
class ConfigurationGraph():
    def __init__(self, configDimensionality, positionDimensionality, initNode):
        self.nodes = list()
        self.edges = dict()
        self.configDim = configDimensionality
        self.posDim = positionDimensionality
        self.nodes.append(initNode)
    
    # Add a new configuration node to the graph, as well as a new edge
    # Between the node and it's parent
    def addNode(self,
            parentNode: ConfigurationNode,
            node: ConfigurationNode,
            connector):
        edge = Edge(parentNode, connector)
        self.nodes.append(node)
        self.edges[node] = edge

    # Return the node with the configuration closest to the given position
    def getNearestNode(self, position):
        minDist = np.inf
        nearestNode = None
        for node in self.nodes:
            # The default np norm is l2, i.e. the distance
            dist = np.linalg.norm(np.array(position[:self.posDim]) - np.array(node.config[:self.posDim]))
            if dist < minDist:
                minDist = dist
                nearestNode = node
        return nearestNode

    # Retreive the nodes and edges for rendering
    def getNodeAndEdgeList(self):
        nodeList = []
        for node in self.nodes:
            nodeList.append(node.config)

        edgeList = []
        for node, edge in self.edges.items():
            for connector in edge.connectors:
                edgeList.append(deepcopy(connector))

        return nodeList, edgeList

    # Get a path from the leaf node at the end to the root node
    def getPath(self):
        nodeList = []
        edgeList = []
        node = self.nodes[-1]
        nodeList.append(deepcopy(node))
        while self.nodes.index(node) != 0:
            edge = self.edges[node]
            node = edge.parentNode
            edgeList.extend(deepcopy(edge.connectors))
            nodeList.append(deepcopy(node))

        return nodeList, edgeList


