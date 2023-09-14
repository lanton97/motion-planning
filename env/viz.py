import pygame
from pygame import gfxdraw
import time
# A basic class to handle drawing of obstacles, vehicles and planning motions for user viewing
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 0)
GREY = (128, 128, 128)
WHITE = (255, 255, 255)

class baseRenderer:
    def __init__(self):
        self._width = 600
        self._height = 600
        pygame.init()
        pygame.display.init()
        self._display = pygame.display.set_mode((self._width,self._height))
        self._surface = pygame.Surface((self._width, self._height))
        self._surface.fill(WHITE)
        self._clock = pygame.time.Clock()

    def draw_walls(self, walls):
        for wall in walls:
            gfxdraw.line(self._surface, wall[0][0] + int(self._width/2), wall[0][1] + int(self._width/2), 
                    wall[1][0] + int(self._width/2), wall[1][1] + int(self._width/2), BLACK)

    def draw_pois(self, start_pos, end_pos):
        gfxdraw.filled_circle(self._surface, start_pos[0] + int(self._width/2), start_pos[1] + int(self._width/2), 5, YELLOW)
        gfxdraw.filled_circle(self._surface, end_pos[0] + int(self._width/2), end_pos[1] + int(self._width/2), 5, GREEN)

    def draw_nodes(self, nodes):
        for node in nodes:
            gfxdraw.filled_circle(self._surface, node[0], node[1], 1, GREY)

    def draw_edges(self, lines):
        for line in lines:
            gfxdraw.line(self._surface, line[0][0], line[0][1], line[1][0], line[1][1], GREY)

    def update(self):
        surf = pygame.transform.flip(self._surface, True, False)
        self._display.blit(surf, (0, 0))
        pygame.event.pump()
        self._clock.tick(1)
        pygame.display.flip()

if __name__=='__main__':
    tester = baseRenderer()
    wall1 = [ (0, 0),  
              (60, 0)]
    wall2 = [ (0, 0), 
              (0, 60)]
    wall3 = [ (0, -300),  
              (0, -200)]
    walls = [wall1, wall2, wall3]

    start_pos = [90, 90]
    end_pos = [-90, -90]
    tester.draw_walls(walls)
    tester.draw_pois(start_pos, end_pos)
    tester.update()
    time.sleep(40)

