# Class to encode the arc components
class arcSeg():
    def __init__(self, centre, th1, th2, radius):
        self.centre = centre
        self.th1 = th1
        self.th2 = th2
        self.rad = radius

# Class to encode a straight line segment
class straightLine():
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
