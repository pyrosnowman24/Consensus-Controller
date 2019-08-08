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
        self.linModel = None
        self.path = None


    def fitfunc1(self,x):
      # return self.p[0] + self.p[1]*(x[0]+self.p[2])**2+self.p[3]*(x[1]+self.p[4])**2
      print("")
    def plotResults(self):
        grid_x,grid_y = np.mgrid[0:1:200j,0:1:200j]
        #RBF Method
        # grid_x2,grid_y2 = np.mgrid[-5:5:20j,-5:5:20j]
        RBFi = interp.Rbf(self.data[:,0],self.data[:,1],self.data[:,2], function='cubic', smooth=0)
        # re-grid the data to fit the entire graph
        zi = RBFi(grid_x, grid_y)
        n = 100
        X = np.linspace(0,1,n)
        Y = X
        X,Y = np.meshgrid(X,Y)
        Z1 = np.zeros((n,n))
        Z2 = np.zeros((n,n))
        for x in range(len(X)):
          for y in range(len(Y)):
            Z1[x][y] = self.linModel[0]*X[x,y]+self.linModel[1]*Y[x,y]+self.linModel[2]
            Z2[x][y] = self.SensorFunc.sensor([X[x,y],Y[x,y]])

        print "Model:",'\n'
        print "x=.2,y=.2,z=",RBFi(.2,.2)
        print "x=.75,y=.32,z=",RBFi(.75,.32)
        print "x=.46,y=.78,z=",RBFi(.46,.78)
        print "x=.45,y=.26,z=",RBFi(.45,.26)
        print "x=.16,y=.52,z=",RBFi(.16,.52)
        print "Actual:",'\n'
        print "x=.2,y=.2,z=",self.SensorFunc.sensor([.2,.2])
        print "x=.75,y=.32,z=",self.SensorFunc.sensor([.75,.32])
        print "x=.46,y=.78,z=",self.SensorFunc.sensor([.46,.78])
        print "x=.45,y=.26,z=",self.SensorFunc.sensor([.45,.26])
        print "x=.16,y=.52,z=",self.SensorFunc.sensor([.16,.52])


        fig,(ax1,ax2) = plt.subplots(1,2,figsize=(10,10))
        cf1 = ax1.contour(grid_x, grid_y, zi, 30, linewidths=0.5, colors='k')
        cf1 = ax1.contourf(grid_x, grid_y, zi, 50, cmap=plt.cm.rainbow)
        ax1.scatter(self.path[:,0], self.path[:,1], marker='o', c='b', s=5, zorder=10)
        for region in self.BoundedVoronoi.vor.filtered_regions:
            vertices = self.BoundedVoronoi.vor.vertices[region + [region[0]], :]
            ax1.plot(vertices[:, 0], vertices[:, 1], 'k-')
        ax1.set_xlim([0, 1])
        ax1.set_ylim([0,1])
        cf2 = ax2.contour(X, Y, Z2, 30,vmax = zi.max(), vmin = zi.min(), linewidths=0.5, colors='k')
        cf2 = ax2.contourf(X, Y, Z2, 50,vmax = zi.max(), vmin = zi.min(), cmap=plt.cm.rainbow)
        fig.colorbar(cf1)  # draw colorbar
        ax2.scatter(self.points[:,0], self.points[:,1], marker='o', c='b', s=5, zorder=10)
        for region in self.BoundedVoronoi.vor.filtered_regions:
            vertices = self.BoundedVoronoi.vor.vertices[region + [region[0]], :]
            ax2.plot(vertices[:, 0], vertices[:, 1], 'k-')
        ax2.set_xlim([0, 1])
        ax2.set_ylim([0,1])
        # # Surface plot of RBF estimation vs real sensor function
        # # Plots RBF model of sensor function
        # fig = plt.figure(figsize=plt.figaspect(.5))
        # ax1 = fig.add_subplot(3,1,1, projection='3d')
        # ax1.plot_surface(grid_x,grid_y,zi)
        # ax1.set_xlim(0,1)
        # ax1.set_ylim(0,1)
        # ax1.set_zlim(0,1)
        # # Plots linear model of sensor function
        # ax2 = fig.add_subplot(3,1,2,projection='3d')
        # surf = ax2.plot_surface(X,Y,Z1)
        # ax2.set_xlim(0,1)
        # ax2.set_ylim(0,1)
        # ax2.set_zlim(0,1)
        # # Plots actual sensor function
        # ax3 = fig.add_subplot(3,1,3,projection='3d')
        # surf = ax3.plot_surface(X,Y,Z2)
        # ax3.set_xlim(0,1)
        # ax3.set_ylim(0,1)
        # ax3.set_zlim(0,1)

        # self.BoundedVoronoi.plot_voronoi()

        plt.show()

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

    def updatePath(self):
        if self.path is None:
            self.path = self.points
        else:
            self.path = np.vstack((self.path,self.points))
