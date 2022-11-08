import math
from typing import List
import numpy as np
import matplotlib.pyplot as plt

class SeedSegment:

    size = 15

    def __init__(self,m,c) -> None:
        self.points = []
        self.grad = m
        self.intersect = c
        
    def plot_line(self,xd) -> None:
        x = np.array(xd)
        plt.plot(x,self.grad*x + self.intersect)
        plt.show()        


class LineDetector:


    seed_segments = []
    epsilon = 0
    sigma = 0
    Pmin = 15


    def make_seed_segments(lidar_data) -> None:
        '''
        Adds seed segments to seed segment array
        '''
        xs,ys = LineDetector.lidar_2_points(lidar_data)
        flag = True
    
        for i in range(0,len(lidar_data)-LineDetector.Pmin):
            j = i + SeedSegment.size
            m,c = LineDetector.points_2_line(xs[i:j],ys[i:j])
            for point_index in range(i,j):
                if LineDetector.predicted_point_distance([m,c],[xs[point_index],ys[point_index]]) > LineDetector.epsilon:
                    flag = False
                    break
                if LineDetector.seed_line_distance([m,c],[xs[point_index],ys[point_index]]) > LineDetector.sigma:
                    flag = False
                    break
            if flag:
                LineDetector.seed_segments.append(SeedSegment(m,c))
                print("segment detected")


    def predicted_point_distance(seedline,point) -> float:
        '''
        determines where the line between a point and the robot intersects given seed segment line
        '''
        pointline_grad = point[1]/point[0]
        pointline_intercept = 0 # intercept is always 0 as robot is still located at (0,0)
                                # this will be changed once the static assumption is removed

        i_x = (seedline[1] - pointline_intercept)/(pointline_grad - seedline[0])
        i_y = (seedline[1]*seedline[0] - pointline_intercept*pointline_grad)/(seedline[0] - pointline_grad)

        return math.sqrt((point[0] - i_x)**2 + (point[1] - i_y)**2)

    def seed_line_distance(line,point):
        '''
        determines a points euclidean distance from the seed segment line
        '''
        perpendicular_grad = -1/line[0]
        perpendicular_intercept = point[1] - perpendicular_grad*point[0]

        i_x = (perpendicular_intercept - line[1])/(line[0] - perpendicular_grad)
        i_y = (line[1]*perpendicular_grad - line[0]*perpendicular_intercept)/(perpendicular_grad - line[0])
    
        return math.sqrt((point[0] - i_x)**2 + (point[1] - i_y[1]))


    # HELPER FUNCTIONS
    

    def lidar_2_points(lidar_ranges) -> List[List[float]]:
        '''
        Converts raw lidar data to arrays of x and y coordinates relative to scanner
        '''
        xs = []
        ys = []
        for measurement in range(0,len(lidar_ranges)):
            xs.append(lidar_ranges[measurement] * math.sin(math.radians(measurement)))
            ys.append(lidar_ranges[measurement] * math.cos(math.radians(measurement)))

        return xs,ys


    def points_2_line(Xvals,Yvals) -> List[float]:
        '''
        Returns gradient and intercept of least squares regression line of points provided
        '''
        Xpoints = np.array(Xvals)
        Ypoints = np.array(Yvals)
        return (np.linalg.pinv(np.column_stack((Xpoints,np.ones(Xpoints.size))))) @ Ypoints
