from sampling.graphs.base import *

# This class also stores the cumulative cost of each node`
class CostConfigurationGraph(ConfigurationGraph):

    def __init__(self, configDimensionality, positionDimensionality, initNode):
        self.nodes = list()
        self.costs = dict()
        self.edges = dict()
        self.configDim = configDimensionality
        self.posDim = positionDimensionality
        self.nodes.append(initNode)
        self.costs[initNode] = 0
    
    # Add a new configuration node to the graph, as well as a new edge
    # Between the node and it's parent
    def addNode(self,
            parentNode: ConfigurationNode,
            node: ConfigurationNode,
            costFunc,
            connector):
        edge = Edge(parentNode, connector)
        self.nodes.append(node)
        self.edges[node] = edge
        self.costs[node] = costFunc(node.config, parentNode.config) + self.costs[parentNode]

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

    # Get all nodes in a neighbourhood
    def getNeighbourhoodNodes(self, position, delta):
        nodeList= []
        for node in self.nodes:
            # The default np norm is l2, i.e. the distance
            dist = np.linalg.norm(np.array(position[:self.posDim]) - np.array(node.config[:self.posDim]))
            if dist < delta:
                nodeList.append(node)
        return nodeList

    # Get the lowest cost neighbour in a given neighbourhood
    def getLowestCostNeighbour(self, position, delta, initCandidate, initCost, costFunc):
        minCost = initCost
        lowestCostNode = initCandidate
        for node in self.nodes:
            # The default np norm is l2, i.e. the distance
            dist = np.linalg.norm(np.array(position[:self.posDim]) - np.array(node.config[:self.posDim]))
            cost = costFunc(position, node.config) + self.costs[node]

            if dist < delta and cost < minCost:
                lowestCostNode = node
                minCost = cost

        return lowestCostNode

    # Rewire neighbour nodes if it would lower their cost
    def rewireIfCheaper(self, node, potentialChild, costFunc, dynamics, rewireDist):
        if costFunc(node.config, potentialChild.config) + self.costs[node] < self.costs[potentialChild]:
            _, cost, connector = dynamics.sampleWCost(node, potentialChild.config, rewireDist, costFunc)
            edge = Edge(node, connector)
            self.edges[potentialChild] = edge
            self.costs[potentialChild] = costFunc(node.config, potentialChild.config) + self.costs[node]

    def addEdge(self, parentNode, node, connector, costFunc):
        edge = Edge(parentNode, connector)
        self.edges[node] = edge
        self.costs[node] = costFunc(node.config, parentNode.config) + self.costs[parentNode]


    def addNodesNoEdge(self,
            nodes,
            ):
        for node in nodes:
            self.nodes.append(node)
            self.costs[node] = 0 
