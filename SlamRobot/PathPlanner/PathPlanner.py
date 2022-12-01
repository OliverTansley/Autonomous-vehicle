from DataStructures.Tree import Tree
from DataStructures.Tree import Node

from Utils.ArithmeticUtil import *

import random
import math

class PathPlanner:

    N = 40
    found = False
    k = 200
    

    @staticmethod
    def start(start,end)->Tree:
        '''
        RRT with euclidean distance heuristic
        '''
        T:Tree = Tree(start[0],start[1])
        while not(PathPlanner.found):
        
            # Select best node to extend (do while loop)

            randomPos:tuple[int,int] = (int(4*320*random.random()),int(4*240*random.random()))
            nearestNode:Node = T.getClosestNode(randomPos)
            nearestNodescore:float = (math.sqrt((nearestNode.x - end[0])**2 + (nearestNode.y - end[1])**2) - math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2))
            rand_bound = random.random()

            while nearestNodescore < rand_bound:
                randomPos:tuple[int,int] = (int(4*320*random.random()),int(4*240*random.random()))
                nearestNode:Node = T.getClosestNode(randomPos)
                nearestNodescore:float = ((math.exp(-point_2_point_distance(randomPos,end)/PathPlanner.k)))
                print(str(point_2_point_distance(randomPos,end)/PathPlanner.k),":",nearestNodescore)
                rand_bound:float = random.random()
            
            # Extend tree with new node

            T.addNode(nearestNode,Node(randomPos[0],randomPos[1]))
            if point_2_point_distance(randomPos,(end[0],end[1])) < 50:
                PathPlanner.found = True
        return T
