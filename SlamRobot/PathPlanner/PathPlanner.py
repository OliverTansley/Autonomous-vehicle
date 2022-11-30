from DataStructures.Tree import Tree
from DataStructures.Tree import Node

from Utils.ArithmeticUtil import *

import random
import math

class PathPlanner:

    N = 40
    found = False
    k = 4

    @staticmethod
    def start(start,end)->Tree:
        '''
        Basic tree building algorithm
        '''
        T = Tree(start[0],start[1])
        for _ in range(40):

            # Select best node to extend

            randomPos:tuple[int,int] = (int(4*320*random.random()),int(4*240*random.random()))
            nearestNode:Node = T.getClosestNode(randomPos)
            nearestNodescore = (math.sqrt((nearestNode.x - end[0])**2 + (nearestNode.y - end[1])**2) - math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2))
            rand_bound = random.random()

            while nearestNodescore < rand_bound:
                randomPos:tuple[int,int] = (int(4*320*random.random()),int(4*240*random.random()))
                nearestNode:Node = T.getClosestNode(randomPos)
                nearestNodescore = 1 -((100*math.exp(-point_2_point_distance(randomPos,end)/PathPlanner.k))/100)
                rand_bound = random.random()
            
            # Extend tree with new node

            T.addNode(nearestNode,Node(randomPos[0],randomPos[1]))
            if point_2_point_distance(randomPos,(end[0],end[1])) < 50:
                PathPlanner.found = True
        return T
