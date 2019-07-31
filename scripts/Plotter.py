#!/usr/bin/env python
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from matplotlib import cm
import scipy.interpolate as interp

class PlotData:
    def __init__(self,BoundedVoronoi,SensorFunc):
        self.points = np.zeros((4,2))
        self.data = None
        self.BoundedVoronoi = BoundedVoronoi
        self.SensorFunc = SensorFunc


    def fitfunc1(self,x):
      # return self.p[0] + self.p[1]*(x[0]+self.p[2])**2+self.p[3]*(x[1]+self.p[4])**2
      print("")
    def plotResults(self):
        grid_x,grid_y = np.mgrid[0:1:200j,0:1:200j]
        #RBF Method
        # grid_x2,grid_y2 = np.mgrid[-5:5:20j,-5:5:20j]
        RBFi = interp.Rbf(self.data[:,0],self.data[:,1],self.data[:,2], function='inverse', smooth=0)
        # re-grid the data to fit the entire graph
        zi = RBFi(grid_x, grid_y)

        # Contour plot of grid data
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        # contour the gridded data, plotting dots at the nonuniform data points.
        CS = plt.contour(grid_x, grid_y, zi, 30, linewidths=0.5, colors='k')
        CS = plt.contourf(grid_x, grid_y, zi, 50, cmap=plt.cm.rainbow)
        plt.colorbar()  # draw colorbar
        plt.xlim(0, 1)
        plt.ylim(0, 1)

        X = np.linspace(0,1,60)
        Y = X
        X,Y = np.meshgrid(X,Y)
        Z2 = np.zeros((60,60))
        for x in range(len(X)):
          for y in range(len(Y)):
            Z2[x][y] = self.SensorFunc.sensor([X[x,y],Y[x,y]])
        # contour plot of RBF estimation
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        # contour the gridded data, plotting dots at the nonuniform data points.
        CS = plt.contour(X, Y, Z2, 30, linewidths=0.5, colors='k')
        CS = plt.contourf(X, Y, Z2, 50, cmap=plt.cm.rainbow)
        plt.colorbar()  # draw colorbar
        plt.xlim(0, 1)
        plt.ylim(0, 1)

        # Surface plot of RBF estimation vs real sensor function

        fig = plt.figure(figsize=plt.figaspect(.5))
        ax1 = fig.add_subplot(1,1,1, projection='3d')
        ax1.plot_surface(grid_x,grid_y,zi)
        ax1.set_xlim(0,1)
        ax1.set_ylim(0,1)
        ax1.set_zlim(0,1)
        ax2 = fig.add_subplot(1,2,2,projection='3d')
        surf = ax2.plot_surface(X,Y,Z2)
        ax2.set_xlim(0,1)
        ax2.set_ylim(0,1)
        ax2.set_zlim(0,1)
        plt.tight_layout(rect=[0, 0, 1, 0.95])

        self.BoundedVoronoi.plot_voronoi()

        plt.show()

        # Plot Voronoi


    def updateData(self,data):
        if self.data is None:
            self.data = data
        elif(len(self.data)>=300):
            for row in self.data:
                if list(row) == data:
                    return None
            self.data = np.delete(self.data,0,0)
            self.data = np.vstack((self.data,data))
        else:
            self.data = np.vstack((self.data,data))
