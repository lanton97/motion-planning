import numpy as np
from copy import deepcopy

class ConfigurationNode():
    def __init__(self, configuration):
        self.config = configuration

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
            node: ConfigurationNode):
        self.nodes.append(node)
        self.edges[node] = parentNode

    # Return the node with the configuration closest to the given position
    def getNearestNode(self, position):
        minDist = np.inf
        nearestNode = None
        for node in self.nodes:
            # The default np norm is l2, i.e. the distance
            dist = np.linalg.norm(np.array(position) - np.array(node.config[:self.posDim]))
            if dist < minDist:
                minDist = dist
                nearestNode = node
        return nearestNode

    def getNodeAndEdgeList(self):
        nodeList = []
        for node in self.nodes:
            nodeList.append(node.config)

        edgeList = []
        for node, parent in self.edges.items():
            edgeList.append((deepcopy(node.config), deepcopy(parent.config)))

        return nodeList, edgeList

