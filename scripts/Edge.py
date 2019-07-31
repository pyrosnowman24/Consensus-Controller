#!/usr/bin/python
import numpy as np
from scipy.stats import multivariate_normal
from scipy import optimize


class Edge:
  def __init__(self):
    self.data = np.zeros(([4,3]))
    self.edgeP = np.array((1,1,1))
    self.p0 = np.array([1,1,1])

  def linearApprox(self):
    for row in self.data:
        if np.sum(row) == 0:
            return None
    xs = self.data[:,0]
    ys = self.data[:,1]
    zs = self.data[:,2]
    tmp_A = []
    tmp_b = []
    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b
    self.edgeP = [fit.item(0),fit.item(1),fit.item(2)]
