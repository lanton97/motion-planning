import numpy as np

def distanceCost(config1, config2):
    return np.linalg.norm(config1 - config2)
