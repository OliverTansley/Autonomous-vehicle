import random
import math
from DataStructures.Tree import Tree
from DataStructures.Tree import Node

class PathPlanner:

    N = 40
    found = False

    @staticmethod
    def start(start,end)->Tree:
        '''Basic RRT algorithm'''
        T = Tree(start[0],start[1])
        while not(PathPlanner.found):
            randomPos:tuple[int,int] = (int(4*320*random.random()),int(4*240*random.random()))
            nearestNode:Node = T.getClosestNode(randomPos)
            T.addNode(nearestNode,Node(randomPos[0],randomPos[1]))

            if math.sqrt((randomPos[0] - end[0])**2 + (randomPos[1] - end[1])**2) < 50:
                PathPlanner.found = True
        return T
