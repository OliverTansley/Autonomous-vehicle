import math
from typing import List
import numpy as np
import matplotlib.pyplot as plt
from scipy.odr import *
class SeedSegment:

    size = 10
   
    def __init__(self,m,c,X,Y,x,y,xpnts,ypnts) -> None:
        
        self.grad = m
        self.intersect = c
        self.max_X = X
        self.max_Y = Y
        self.min_x = x
        self.min_y = y
        self.points_x = xpnts
        self.points_y = ypnts

    def plot_line(self) -> None:
        
        x = [x/100 for x in range(int(self.min_x*100),int(self.max_X*100))]
        x = np.array(x)
        
        plt.plot((x),self.grad*(x) + self.intersect)
        plt.show()


class LineDetector:


    seed_segments = []
    epsilon = 1000000
    sigma = 100000
    Pmin = 20


    def make_seed_segments(lidar_data) -> None:
        '''
        Adds seed segments to seed segment array
        '''
        np.seterr('raise')

        xs,ys = LineDetector.lidar_2_points(lidar_data)
        flag = True
    
        for i in range(0,len(lidar_data)- LineDetector.Pmin):
            j = i + SeedSegment.size
            m,c = LineDetector.total_least_squares(xs[i:j],ys[i:j])
            print(LineDetector.total_least_squares(xs[i:j],ys[i:j]))
            Ppoints = []
            for point_index in range(i,min(j,len(lidar_data)-LineDetector.Pmin)):

                Ppoints.append(LineDetector.predicted_point_distance([m,c],[xs[point_index],ys[point_index]]))
                
                if LineDetector.predicted_point_distance([m,c],[xs[point_index],ys[point_index]]) > LineDetector.epsilon:
                    flag = False
                    print("ppd")
                    break
                if LineDetector.seed_line_distance([m,c],[xs[point_index],ys[point_index]]) > LineDetector.sigma:
                    flag = False
                    print("sld")
                    break
                
            if flag:
                LineDetector.seed_segments.append(SeedSegment(m,c,max(xs[i:j]),max(ys[i:j]),min(xs[i:j]),min(ys[i:j]),xs[i:j],ys[i:j]))

                plt.plot(xs,ys)
                SeedSegment(m,c,max(xs[i:j]),max(ys[i:j]),min(xs[i:j]),min(ys[i:j]),xs[i:j],ys[i:j]).plot_line()
                
                
                


    def predicted_point_distance(seedline,point) -> float:
        '''
        determines where the line, between a point and the robot intersects a given seed segment line
        '''

        pointline_grad = point[1]/(point[0] +0.00000001) # TODO remove static assumption
        pointline_intercept = 0 # intercept is always 0 as robot is still located at (0,0)
                                # this will be changed once the static assumption is removed
        
        i_x = (seedline[1] - pointline_intercept)/(pointline_grad - seedline[0])
        i_y = (seedline[0]*pointline_intercept - seedline[1]*pointline_grad)/(seedline[0] - pointline_grad)
       
        return math.sqrt((point[0] - i_x)**2 + (point[1] - i_y)**2)

    def seed_line_distance(line,point):
        '''
        determines a points euclidean distance from the seed segment line
        '''
        
        perpendicular_grad = -1/line[0]
        perpendicular_intercept = point[1] - perpendicular_grad*point[0]

        i_x = (perpendicular_intercept - line[1])/(line[0] - perpendicular_grad)
        i_y = (line[1]*perpendicular_grad - line[0]*perpendicular_intercept)/(perpendicular_grad - line[0])
    
        return math.sqrt((point[0] - i_x)**2 + (point[1] - i_y)**2)


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
        Xpoints = Xpoints[np.isfinite(Xpoints)]
        Ypoints = Ypoints[np.isfinite(Ypoints)]
        
        
        return (np.linalg.pinv(np.column_stack((Xpoints,np.ones(Xpoints.size))))) @ Ypoints

    def total_least_squares(Xvals,Yvals,atol=1e-13, rtol=0) -> List[float]:
        def f(B, x):
            '''Linear function y = m*x + b'''
            return B[0]*x + B[1]
        Xvals = np.array(Xvals)
        Yvals = np.array(Yvals)
        linear = Model(f)
        mydata = RealData(Xvals, Yvals)
        myodr = ODR(mydata, linear, beta0=[1., 2.])
        myoutput = myodr.run()

        return myoutput.beta