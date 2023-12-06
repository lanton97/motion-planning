import numpy as np
from viz.viz import BaseContinuousRenderer
import time
from copy import deepcopy

# A simple base planner acting as a virtual class
class BaseSamplingPlanner():
    def __init__(self,
            initConfig,
            environment,
            ):

        self.initConfig = initConfig
        self.env = environment
        self.renderer = BaseContinuousRenderer()
        
    def plan(self):
        raise NotImplementedError

    def render(self, graph):
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = graph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines(edgeList)
        self.renderer.update()
            


