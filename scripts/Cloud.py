import numpy as np
from scipy import optimize
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
from skmonaco import mcquad, mcmiser
import time
import scipy.interpolate as interp

class Cloud:
    def __init__(self,sim):
        self.data = None
        self.model = None
        self.sim = sim

    def updateModel(self):
        if self.data is None or isinstance(self.data,list):
            print(self.data)
            return None
        grid_x,grid_y = np.mgrid[0:1:20j,0:1:20j]
        #RBF Method
        RBFi = interp.Rbf(self.data[:,0],self.data[:,1],self.data[:,2], function='cubic', smooth=0)
        self.model = RBFi

    def updateData(self,data):
        if self.data is None:
            self.data = data
            return None
        if isinstance(self.data,list) is False:
            for row in self.data:
                for value in np.subtract(list(row),data):
                    if value <= .0005 and value >= -.0005:
                        return None
        if(len(self.data)>=300):
            # self.data = np.delete(self.data,0,0)
            self.data = np.vstack((self.data,data))
        else:
            self.data = np.vstack((self.data,data))

    def findPositions(self,points):
        self.sim.update_all(points,self.model)
        return self.sim.run()
