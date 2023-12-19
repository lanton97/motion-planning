import numpy as np
from sampling.base import *
from sampling.graphs.base import *
from vehicles.ext.dubins import getClosedFormPath

# This class encodes the dynamics of a dubins car
# The dubin's path consists of 6 possible combinations
# RSR, RSL, LSR, LSL, RLR, LRL, with R standing for
# right, L standing for left, and S for straight
# Dubin's proved that the shortest curve connecting
# two points will consist of these combinations
# and turns of the minimum radius
# We reuse some code detailed here: https://github.com/fgabbert/dubins_py/blob/master/dubins_py.py
# but modify it in order to change the path description from a series of points to endpoints and
# turn components
class dubinsCar():
    def __init__(self,
            turn_radius=3.0,
            ):
        self.turnRad = turn_radius

    def sample(self, nearNode, targetConfiguration, delta):
        # The translation from near node to target node
        translation = targetConfiguration[:2] - np.array(nearNode.config[:2])
        # We get the magnitude of the translation
        lenTrans = np.linalg.norm(translation)
        # Get the new translation based on the max delta
        newTranslation = (translation/lenTrans) * min(delta, lenTrans)
        newConfig = ConfigurationNode(np.array([nearNode.config[0] + newTranslation[0], nearNode.config[1] + newTranslation[1], targetConfiguration[2]]))
        connector = getClosedFormPath(nearNode.config, newConfig.config, self.turnRad)

        return newConfig, connector

    def sampleWCost(self, nearNode, targetConfiguration, delta, costFunc):
        # The translation from near node to target node
        translation = targetConfiguration[:2] - np.array(nearNode.config)[:2]
        # We get the magnitude of the translation
        lenTrans = np.linalg.norm(translation)
        # Get the new translation based on the max delta
        newTranslation = (translation/lenTrans) * min(delta, lenTrans)
        # Create the new node with the relevant configuration
        newConfig = ConfigurationNode(np.array([nearNode.config[0] + newTranslation[0], nearNode.config[1] + newTranslation[1], targetConfiguration[2]]))
        connector = getClosedFormPath(nearNode.config, newConfig.config, self.turnRad)

        cost = costFunc(nearNode.config, newConfig.config)

        return newConfig, cost, connector

    def getRandomOrientation(self, size=1):
        if size == 1:
            return [np.random.uniform(0,2*np.pi)]
        return np.random.uniform([0]*size,[2*np.pi]*size, size=size).reshape(-1,1)
