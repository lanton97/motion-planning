import numpy as np

# A simple distance cost metric
def distanceCost(config1, config2):
    return np.linalg.norm(config1 - config2)
