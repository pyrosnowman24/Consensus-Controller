#!/usr/bin/env python
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from matplotlib import cm

class PlotData:
    def __init__(self,BoundedVoronoi,SensorFunc):
        self.points = np.zeros((4,2))
        self.data = None
        self.p = None
        self.BoundedVoronoi = BoundedVoronoi
        self.SensorFunc = SensorFunc


    def fitfunc1(self,x):
      return self.p[0] + self.p[1]*(x[0]+self.p[2])**2+self.p[3]*(x[1]+self.p[4])**2

    def plotResults(self):
        X = np.linspace(0,1,60)
        Y = X
        X,Y = np.meshgrid(X,Y)
        Z1 = np.zeros((60,60))
        Z2 = np.zeros((60,60))
        for x in range(len(X)):
          for y in range(len(Y)):
            Z1[x][y] = self.fitfunc1([X[x,y],Y[x,y]])
            Z2[x][y] = self.SensorFunc.sensor([X[x,y],Y[x,y]])

        self.BoundedVoronoi.plot_voronoi()
        fig = plt.figure(figsize=plt.figaspect(.5))
        bx = fig.add_subplot(1,2,1,projection='3d')
        # bx.scatter(self.data[:,0], self.data[:,1], marker = 'x',color = 'r', zs=0, zdir='z')
        Z3 = np.subtract(Z1,Z2)
        surf = bx.plot_surface(X,Y,Z1, rstride=1, cstride=1, cmap=cm.coolwarm,linewidth=0, antialiased=False)
        bx.set_xlim(0,1)
        bx.set_ylim(0,1)
        bx = fig.add_subplot(1,2,2,projection='3d')
        surf = bx.plot_surface(X,Y,Z2, rstride=1, cstride=1, cmap=cm.coolwarm,linewidth=0, antialiased=False)
        bx.set_xlim(0,1)
        bx.set_ylim(0,1)
        # fig.tight_layout()
        plt.show()

    def updateData(self,data):
        if self.data is None:
            self.data = data
        else:
            self.data = np.vstack((self.data,data))
