import numpy as np
import vehicles.base_vehicle
from sampling.base import *
from sampling.graphs.base import *
from common.geom import straightLine

# This class encodes the dynamics of a particle vehicle
class particleDynamics2D():

    def sample(self, nearNode, targetConfiguration, delta):
        # The translation from near node to target node
        translation = targetConfiguration - np.array(nearNode.config)
        # We get the magnitude of the translation
        lenTrans = np.linalg.norm(translation)
        # Get the new translation based on the max delta
        newTranslation = (translation/lenTrans) * min(delta, lenTrans)
        # Create the new node with the relevant configuration
        newConfig = ConfigurationNode(np.array([nearNode.config[0] + newTranslation[0], nearNode.config[1] + newTranslation[1]]))
        connector = straightLine(nearNode.config, newConfig.config)

        return newConfig, [connector]

    def sampleWCost(self, nearNode, targetConfiguration, delta, costFunc):
        # The translation from near node to target node
        translation = targetConfiguration - np.array(nearNode.config)
        # We get the magnitude of the translation
        lenTrans = np.linalg.norm(translation)
        # Get the new translation based on the max delta
        newTranslation = (translation/lenTrans) * min(delta, lenTrans)
        # Create the new node with the relevant configuration
        newConfig = ConfigurationNode(np.array([nearNode.config[0] + newTranslation[0], nearNode.config[1] + newTranslation[1]]))

        cost = costFunc(nearNode.config, newConfig.config)
        connector = straightLine(nearNode.config, newConfig.config)

        return newConfig, cost, [connector]

    def getRandomOrientation(self):
        return []


        


