import numpy as np
from sampling.graphs.cost_graph import *
from collisionCheckers.coll import CollChecker
from sampling.rrt import *
from sampling.costs.dist import distanceCost

# An implementation of the Fast Marching Tree Algorithm
class FMT(RRT):
    def __init__(self,
            environment,
            deltaConf,
            collChecker=CollChecker,
            vehicleDynamics=None,
            finishDist=3,
            costFunction=distanceCost,
            ):
        super().__init__(environment, deltaConf,collChecker, vehicleDynamics)
        self.nearMemory = dict()
        self.finishDist = finishDist
        self.costFunc = costFunction
        self.configNodeType = ConfigurationNode
        self.configGraphType = CostConfigurationGraph


    def plan(self, numSamples, render=True):
        randomPos = self.env.getRandomPosition(size=numSamples)
        randomOr  = self.dynamics.getRandomOrientation(size=numSamples)
        if len(randomOr)==0:
            nodes = [self.configNodeType(i) for i in randomPos]
        else:
            configs = np.concatenate((randomPos, randomOr), axis=1)
            nodes = [self.configNodeType(i) for i in configs]

        initNode = self.configNodeType(self.initConfig)
        graph = self.configGraphType(len(self.initConfig), self.env.dim, initNode)

        randOrient = self.dynamics.getRandomOrientation()
        endConfig = np.array([*self.env.endPos, *randOrient])
        endNode = self.configNodeType(endConfig)
        nodes.append(endNode)


        graph.addNodesNoEdge(nodes)
        unvisited = {i for i in nodes}
        unvisited.add(initNode)
        VOpen = {initNode}
        VClosed = set()

        z = initNode

        image_data = []
        while z is not endNode and len(VOpen) != 0:
            openNewV = set()
            Nz = set(graph.getNeighbourhoodNodes(z.config, self.delConf)).intersection(unvisited)
            self.save(Nz, z)
            Xnear = Nz.intersection(unvisited)
            for x in Xnear:
                Nx = set(graph.getNeighbourhoodNodes(x.config, self.delConf))
                Nx.discard(x)
                self.save(Nx, x)
                Ynear = list(Nx.intersection(VOpen))
                if len(Ynear) == 0:
                    break
                yMin = Ynear[np.argmin([graph.costs[y] + self.costFunc(y.config, x.config) for y in Ynear])]
                _, connector = self.dynamics.sample(x, yMin.config, self.delConf)
                if not self.collChecker.checkCollisions(connector, self.env):
                    graph.addEdge(yMin, x, connector, self.costFunc)
                    openNewV.add(x)
                    unvisited.remove(x)

                if render:
                    image_data.append(self.render(graph))

            VOpen.remove(z)
            VOpen.update(openNewV)
            if len(VOpen) == 0:
                print("Search failed")
                break
            VClosed.add(z)
            tmpVOpen = list(VOpen)
            z = tmpVOpen[np.argmin([graph.costs[v] for v in VOpen])]


        if render:
            image_data.append(self.render(graph))
            #image_data.append(self.highlightFinalPath(graph))

        return graph, image_data

    def isConnected(self, z):
        return np.linalg.norm(z.config[:self.env.dim] - self.env.endPos) < self.finishDist

    def save(self, Nz, z):
        self.nearMemory[z] = Nz





