import numpy as np
from utility import cal_Euclidean_dist, cal_Manhattan_dist


# 定义每一个网格的状态
class State(object):
    def __init__(self, loc):
        #backpointer
        self.bp = None
        self.loc = loc
        # h: the path cost from goal to point
        self.h = 0.
        # since there are many paths from goal to current point,
        # k is the minmal value of those path cost
        self.k = 0.
        self.t = "NEW"
        self.is_obstacle = False


    def cost(self, other):
        if self.is_obstacle or other.is_obstacle:
            return np.inf
        return cal_Manhattan_dist(self.loc, other.loc)




