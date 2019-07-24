import numpy as np
from scipy import optimize
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
from skmonaco import mcquad, mcmiser
import time

class Cloud:
    def __init__(self,sim):
        self.p = np.array([1,1,1,1,1])
        self.p0 = np.array([1,1,1,1,1])
        self.data = None
        self.equilValue = .2
        self.equilibrium = False
        self.sim = sim

    def updateModel(self):
        # fitfunc = lambda p,x: p[0] + p[1]*x[:,0]**3+p[2]*x[:,1]**3
        fitfunc = lambda p,x: p[0] + p[1]*(x[:,0]+p[2])**2+p[3]*(x[:,1]+p[4])**2
        errfunc = lambda p,x,y: fitfunc(p,x)-y

        if self.data is None:
            return None
        if isinstance(self.data,list): # Tests if data is a matrix or array, it has to be a matrix to be used
          return None
        if self.data.shape[0]<len(self.p0): # Tests if data has more points then variables in p, it must be higher
          return None

        p1,success = optimize.leastsq(errfunc,self.p0,args=(self.data[:,0:2],self.data[:,-1]))
        test = np.subtract(p1,self.p)
        test = np.absolute(test)
        if any(test>self.equilValue):
            self.equilibrium = False
          # The model has not yet reached equilibrium and shouldnt be used
        else:
          self.equilibrium = True
          # The model has reached equilibrium, therefore it should of properly modeled the sensor function and can be used
        self.p = p1
        self.p0 = p1

    def updateData(self,data):
        if self.data is None:
            self.data = data
        else:
            self.data = np.vstack((self.data,data))

    def findPositions(self,points):
        self.sim.update_all(points,self.p)
        return self.sim.run()
