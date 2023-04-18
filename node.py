import numpy as np
import json
from utility import check_in_area, is_exist_same_loc,check_in_area_with_R
from utility import cal_Euclidean_dist, cal_Manhattan_dist


class Node(object):
    def __init__(self, loc):
        self.parent = None
        self.loc = loc
        # total cost: f = g + h
        self.f = 0.
        # g: the cost from start point to current point
        self.g = 0.
        # h: the cost from current point to goal
        self.h = 0.

    def __eq__(self, other):
        return np.array_equal(self.loc, other.loc)

    def __hash__(self):
        return hash((self.loc[0], self.loc[1], self.loc[2]))

    def cost(self, new_loc):
        return cal_Manhattan_dist(self.loc, new_loc)

    def find_next_nodes(self,goal,vertex,edge):
        available = []
        # 根据坐标得到当前点的index，然后再edge中取得邻域的其他点和坐标
        index = str(np.where((np.isin(vertex,self.loc)).all(axis=1))[0][0])
        for ner_vertex in edge.get(index):
            loc = vertex[int(ner_vertex[0])]
        # for direction in self.walk_dirs:
        #     loc = self.loc + direction

#             if (not check_in_area(loc, self.space_boundary)) or is_exist_same_loc(loc, cspace):
# #                print("OverStep or exist same loc")
#                 continue
#             else:
            n = Node(loc)
            n.parent = self
            # g = g_parent + g_move
            n.g = self.g + cal_Euclidean_dist(self.loc,n.loc) + cal_Manhattan_dist(self.loc[2], n.loc[2],unit_c=3)
            n.h = cal_Manhattan_dist(n.loc, goal)
            n.f = n.g + n.h
            available.append(n)
        return available

class Agent(Node):
    def __init__(self):
        super(Agent,self).__init__(None)
        self.name = ''
        self.id = 0
        self.color = 0



