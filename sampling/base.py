import numpy as np
from viz.viz import BaseRenderer

# A simple base planner acting as a virtual class
class BaseSamplingPlanner():
    def __init__(self):
        pass
        
    def plan(self):
        raise NotImplementedError

    def render(self):
        pass

class ConfigurationNode():
    def __init__(self, configuration):
        self.config = configuration

# This graph is used to hold the configurations and connections
# for sampling based planners
class ConfigurationGraph():
    nodes = list()
    edges = set()
    def __init__(self, configDimensionality, positionDimensionality, initNode):
        self.configDim = configDimensionality
        self.posDim = positionDimensionality
        self.nodes.append(initNode)
    
    # Add a new configuration node to the graph, as well as a new edge
    # Between the node and it's parent
    def addNode(self,
            parentNode: ConfigurationNode,
            node: ConfigurationNode):
        self.nodes.append(node)
        self.edges.add((parentNode, Node))

    # Return the node with the configuration closest to the given position
    def getNearestNode(self, position):
        minDist = inf
        nearestNode = None
        for node in self.nodes:
            # The default np norm is l2, i.e. the distance
            dist = np.linalg.norm(position - node.config[:self.posDim])
            if dist < minDist:
                minDist = dist
                nearestNode = node
        return nearestNode





