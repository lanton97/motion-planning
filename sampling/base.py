import numpy as np

class BaseSamplingPlanner():
    def __init__(self,
            environment,
            dynamics,
            ):
        self.env = environment
        self.dyn = dynamics

    def plan(self):
        raise NotImplementedError

class ConfigurationNode():
    def __init__(self, configuration):
        self.config = configuration


class ConfigurationGraph():
    nodes = list()
    edges = set()
    def __init__(self, dimensionality, initNode):
        self.dim = dimensionality
        self.nodes.append(initNode)
    
    def addNode(self,
            parentNode: ConfigurationNode,
            node: ConfigurationNode):
        self.nodes.append(node)
        self.edges.add((parentNode, Node))

    def getNearestNode(self, position):
        minDist = inf
        nearestNode = None
        for node in self.nodes:
            dist = np.linalg.norm(position - node.config[:self.dim])
            if dist < minDist:
                minDist = dist
                nearestNode = node
        return nearestNode





