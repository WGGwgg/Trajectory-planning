import numpy as np
from node import Node
from state import State
from utility import cal_Euclidean_dist, cal_Manhattan_dist

class Astar:
    def __init__(self,vertex,edge, start, goal):
        """
        start and goal are in degree
        """
        self.vertex = vertex
        self.edge = edge
        self.real_start = start
        self.real_goal = goal
        self.start = start
        self.goal = goal
        # self.cspace = cspace
        # self.space_boundary = space_boundary
        # self.walk_dirs = walk_dirs

    def search(self):
        openset = set()                         # 存放可访问的节点
        closedset = set()                       # 存放不可访问的节点

        start = Node(self.start)   # 起始节点
        start.h = start.cost(self.goal)
        openset.add(start)

        final_node = None
        reached = False
        while openset and not reached:
            q = min(openset, key=lambda o:o.f)
            # print(q.loc)
            openset.remove(q)
            final_node = q
            successors = q.find_next_nodes(self.goal,vertex=self.vertex,edge=self.edge)
            # print(len(successors))
            for successor in successors:
                if np.array_equal(successor.loc, self.goal):
                    final_node = successor
                    reached = True

                    break
                if successor in closedset:
                    continue
                if successor in openset:
                    for s in openset:
                        if np.array_equal(s.loc, successor.loc) and successor.f < s.f:
                            s.parent = successor.parent
                            s.g = successor.g
                            s.h = successor.h
                            s.f = successor.f
                            break
                else:
                    openset.add(successor)
            closedset.add(q)
        path = self.real_goal
        while final_node.parent is not None:
            path = np.vstack((final_node.loc, path))
            final_node = final_node.parent
        path = np.vstack((self.real_start, path))
        return path.T
