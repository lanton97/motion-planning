import pygame
from pygame import gfxdraw
import time
from copy import deepcopy
from common.geom import *
from common.util import *

# Colour Presets
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 0)
GREY = (128, 128, 128)
WHITE = (255, 255, 255)

# A basic class to handle drawing of obstacles, vehicles and planning motions for user viewing
class BaseContinuousRenderer:
    # Create default PyGame drawing objects and windows
    def __init__(self, width=600, height=600):
        self._width = width
        self._height = height
        pygame.init()
        pygame.display.init()
        self._display = pygame.display.set_mode((self._width,self._height))
        self._surface = pygame.Surface((self._width, self._height))
        self._surface.fill(WHITE)
        self._clock = pygame.time.Clock()

    # This function takes a list of lists, and draws them as walls
    # Each wall should have one list for a start point and one
    # for and end point
    def draw_walls(self, walls):
        walls_tf = self._translate_lines(deepcopy(walls))
        for wall in walls_tf:
            gfxdraw.line(self._surface, int(wall.p1[0]), int(wall.p1[1]), int(wall.p2[0]), int(wall.p2[1]), BLACK)

    # Draw the start and end position, each a list of start and end points
    def draw_pois(self, start_pos, end_pos):
        start_pos_tf = self._translate_point(deepcopy(start_pos))
        end_pos_tf = self._translate_point(deepcopy(end_pos))
        gfxdraw.filled_circle(self._surface, int(start_pos_tf[0]), int(start_pos_tf[1]), 5, YELLOW)
        gfxdraw.filled_circle(self._surface, int(end_pos_tf[0]), int(end_pos_tf[1]), 5, GREEN)

    # Draw nodes for the motion planning
    def draw_nodes(self, nodes):
        nodes_tf = self._translate_points(deepcopy(nodes))
        for node in nodes_tf:
            gfxdraw.filled_circle(self._surface, int(node[0]), int(node[1]), 1, GREY)

    def draw_lines_and_curves(self, connectors):
        connectors_tf = deepcopy(connectors)
        for connector in connectors:
            if type(connector) is arcSeg:
                connector = self._translate_arc_seg(connector)
                gfxdraw.arc(self._surface, int(connector.centre[0]), int(connector.centre[1]), int(connector.rad), int(rad2Deg(connector.th1)), int(rad2Deg(connector.th2)), GREY)
            else:
                connector = self._translate_line(connector)
                gfxdraw.line(self._surface, int(connector.p1[0]), int(connector.p1[1]), int(connector.p2[0]), int(connector.p2[1]), GREY)


    # Draw straight edge connections between nodes
    # Requires the start node and end node positions
    def draw_lines(self, lines):
        lines_tf = self._translate_lines(deepcopy(lines))
        for line in lines_tf:
            gfxdraw.line(self._surface, int(line.p1[0]), int(line.p1[1]), int(line.p2[0]), int(line.p2[1]), GREY)

    # Update the windows
    def update(self):
        surf = pygame.transform.flip(self._surface, True, False)
        self._display.blit(surf, (0, 0))
        pygame.event.pump()
        pygame.display.flip()

    def clear(self):
        self._surface.fill(WHITE)

    def pull_array(self):
        img_data = pygame.surfarray.array3d(self._surface)
        img_data = img_data.swapaxes(0,1)
        return img_data

    ####################### Utility Functions ###########################
    # The following functions translate the positions of objects from a zero-centred 
    # coordinate system into the screen coordinates for drawing

    def _translate_arc_segs(self, arc_segs):
        for arcSeg in arc_segs:
            arcSeg = self._translate_arc_seg(arcSeg)
        return arc_segs

    
    def _translate_arc_seg(self, arc_seg):
        centre = arc_seg.centre
        centre[0] += int(self._width / 2)
        centre[1] += int(self._height / 2)
        arc_seg.centre = centre
        return arc_seg


    def _translate_lines(self, lines):
        for line in lines:
            line = self._translate_line(line)
        return lines

    def _translate_line(self, line):
        line.p1[0] += int(self._width / 2)
        line.p1[1] += int(self._height / 2)
        line.p2[0] += int(self._width / 2)
        line.p2[1] += int(self._height / 2)
        return line

    def _translate_points(self, points):
        for point in points:
            point = self._translate_point(point)
        return points

    def _translate_point(self, point):
        point[0] += int(self._width / 2)
        point[1] += int(self._height / 2)
        return point

if __name__=='__main__':
    tester = baseContinuousRenderer()
    wall1 = [ [0, 0],  
              [60, 0]]
    wall2 = [ [0, 0], 
              [0, 60]]
    wall3 = [ [0, -300],  
              [0, -200]]
    walls = [wall1, wall2, wall3]

    start_pos = [90, 90]
    end_pos = [-90, -90]
    tester.draw_walls(walls)
    tester.draw_pois(start_pos, end_pos)
    tester.update()
    time.sleep(40)

