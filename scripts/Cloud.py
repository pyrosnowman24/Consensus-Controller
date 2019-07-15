import numpy as np
from scipy import optimize

class Cloud:
    def __init__(self):
        self.p = np.array([1,1,1])
        self.p0 = np.array([1,1,1])
        self.data = None
        self.equilValue = .01
        self.equilibrium = False

    def updateModel(self):
        fitfunc = lambda p,x: p[0] + p[1]*x[:,0]**3+p[2]*x[:,1]**3
        errfunc = lambda p,x,y: fitfunc(p,x)-y

        if isinstance(self.data,list): # Tests if data is a matrix or array, it has to be a matrix to be used
          return None
        if self.data.shape[0]<len(self.p0): # Tests if data has more points then variables in p, it must be higher
          return None

        p1,success = optimize.leastsq(errfunc,self.p0,args=(self.data[:,0:2],self.data[:,-1]))
        test = np.subtract(p1,self.p)
        test = np.absolute(test)
        if any(test>self.equiValue):
          self.equilibrium = False
          # The model has not yet reached equilibrium and shouldnt be used
        else:
          self.equilibrium = True
          # The model has reached equilibrium, therefore it should of properly modeled the sensor function and can be used
        self.p = p1

    def updateData(self,data):
        if self.data is None:
          self.data = data
        else:
          self.data = np.vstack((self.data,data))
