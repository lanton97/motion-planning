import numpy as np
from viz.viz import BaseContinuousRenderer
import time
from copy import deepcopy
from viz.viz import *

# A simple base planner acting as a virtual class
class BaseSamplingPlanner():
    def __init__(self,
            environment,
            vehicleDynamics,
            ):
        self.env = environment
        self.renderer = BaseContinuousRenderer()
        self.dynamics = vehicleDynamics()
        randOrient = self.dynamics.getRandomOrientation()
        initConfig = np.array([*self.env.startPos, *randOrient])
        self.initConfig = initConfig
        
    def plan(self):
        raise NotImplementedError

    def render(self, graph):
        self.renderer.clear()
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = graph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        self.renderer.update()
        img_data = self.renderer.pull_array()
        return img_data
            
    def highlightFinalPath(self, forwardGraph):
        self.renderer.clear()
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = forwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        _, finalEdges = forwardGraph.getPath()
        self.renderer.draw_lines_and_curves(finalEdges, colour=RED)
        self.renderer.update()
        img_data = self.renderer.pull_array()
        return img_data


