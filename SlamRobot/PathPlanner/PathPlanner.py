from DataStructures.Tree import Tree
from DataStructures.Tree import Node

from Utils.ArithmeticUtil import *

import random
import math

class PathPlanner:

    N:int = 40
    found:bool = False
    k:int = 200


    @staticmethod
    def start(start: tuple[int,int],end: tuple[int,int])->Tree:
        '''
        RRT with euclidean distance heuristic
        '''
        T:Tree = Tree(start[0],start[1])
        while not(PathPlanner.found):
        
            # Select best node to extend (do while loop)

            while True:
                randomPos:tuple[int,int] = (int(4*320*random.random()),int(4*240*random.random()))
                nearestNode:Node = T.getClosestNode(randomPos)
                nearestNodescore:float = ((math.exp(-point_2_point_distance(randomPos,end)/PathPlanner.k)))
                print(str(point_2_point_distance(randomPos,end)/PathPlanner.k),":",nearestNodescore)
                rand_bound:float = random.random()
            
                if nearestNodescore > rand_bound:
                    break

            # Extend tree with new node

            T.addNode(nearestNode,Node(randomPos[0],randomPos[1]))
            if point_2_point_distance(randomPos,(end[0],end[1])) < 50:
                PathPlanner.found:bool = True
        return T
