import numpy as np
from viz.viz import BaseContinuousRenderer
import time
from copy import deepcopy
from viz.viz import *

# A simple base planner acting as a virtual class
# Implements common functions and initializes the common members
class BaseSamplingPlanner():
    def __init__(self,
            environment,
            vehicleDynamics,
            ):
        self.env = environment
        self.renderer = BaseContinuousRenderer()
        self.dynamics = vehicleDynamics()
        # Get a random configuration for the start position
        randOrient = self.dynamics.getRandomOrientation()
        initConfig = np.array([*self.env.startPos, *randOrient])
        self.initConfig = initConfig
        
    def plan(self):
        raise NotImplementedError

    # Render the entire planning scene
    def render(self, graph):
        # Clear the surface so we can rewire the tree if needed
        self.renderer.clear()
        # Draw the environment walls and points of interest
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        # Get the graphs nodes and edges
        nodeList, edgeList = graph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        # Update the surface for vizualization
        self.renderer.update()
        # Return the image data so we can save a gif
        img_data = self.renderer.pull_array()
        return img_data
            
    # same as above, but we also render the path from the final leaf node to
    # the root in red
    def highlightFinalPath(self, forwardGraph, node=None):
        self.renderer.clear()
        self.renderer.draw_walls(self.env.walls)
        self.renderer.draw_pois(self.env.startPos, self.env.endPos)
        nodeList, edgeList = forwardGraph.getNodeAndEdgeList()
        self.renderer.draw_nodes(nodeList)
        self.renderer.draw_lines_and_curves(edgeList)
        # We also get the edges corresponding with the final path
        _, finalEdges = forwardGraph.getPath(node)
        # And highlight them in red
        self.renderer.draw_lines_and_curves(finalEdges, colour=RED)
        self.renderer.update()
        img_data = self.renderer.pull_array()
        return img_data


