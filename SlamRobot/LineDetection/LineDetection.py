import math
import numpy as np
import matplotlib.pyplot as plt

class SeedSegment:

    size = 30

    def __init__(self,m,c) -> None:
        self.points = []
        self.grad = m
        self.intersect = c
        
    def plot_line(self,xd):
        x = np.array(xd)
        plt.plot(x,self.grad*x + self.intersect)
        plt.show()
        


class LineDetector:

    seed_segments = []
    epsilon = 0
    sigma = 0

    def make_seed_segments(lidar_data):
        xs = []
        ys = []
        for measurement in range(0,len(lidar_data)):
            xs.append(lidar_data[measurement] * math.sin(math.radians(measurement)))
            ys.append(lidar_data[measurement] * math.cos(math.radians(measurement)))
            
        m,c = LineDetector.points_2_line(xs,ys)
        current_segment = SeedSegment(m,c)
        current_segment.plot_line(xs)
        plt.scatter(xs,ys)
        plt.show()
        

    def points_2_line(Xvals,Yvals):
        '''
        Returns gradient and intercept of least squares regression line of points provided
        '''
        Xpoints = np.array(Xvals)
        Ypoints = np.array(Yvals)
        return (np.linalg.pinv(np.column_stack((Xpoints,np.ones(Xpoints.size))))) @ Ypoints
