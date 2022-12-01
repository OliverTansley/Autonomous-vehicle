import math
    
def point_2_point_distance(point1,point2) -> float:
    '''
    Get euclidean distance between to points represented as tuples (x1,y1) (x2,y2)
    '''
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1]-point2[1])**2)

