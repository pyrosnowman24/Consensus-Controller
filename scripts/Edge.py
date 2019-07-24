#!/usr/bin/python
import numpy as np
from scipy.stats import multivariate_normal
from scipy import optimize


class Edge:
  def __init__(self):
    self.positions = np.zeros(([4,2]))
    self.data = None
    self.edgeP = np.array((1,1,1))
    self.p0 = np.array([1,1,1])
  def gatherPositions(self,robots):
    self.positions = np.zeros(len(robots))
    for i,robot in enumerate(robots):
      self.positions[i] = robot.pos
  def linearApprox(self):
    fitfunc = lambda p,x: p[0] + p[1]*x[:,0]+p[2]*x[:,1]
    errfunc = lambda p,x,y: fitfunc(p,x)-y
    if self.data is None:
        return None
    if isinstance(self.data,list): # Tests if data is a matrix or array, it has to be a matrix to be used
      return None
    if self.data.shape[0]<len(self.p0): # Tests if data has more points then variables in p, it must be higher
      return None

    p1,success = optimize.leastsq(errfunc,self.p0,args=(self.data[:,0:2],self.data[:,-1]))
    self.edgeP = p1
  def updateData(self,data):
    if self.data is None:
      self.data = data
    else:
      self.data = np.vstack((self.data,data))
