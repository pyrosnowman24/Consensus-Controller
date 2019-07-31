#!/usr/bin/python
import numpy as np
import rospy
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
from skmonaco import mcquad, mcmiser

class Sim:
  def __init__(self,BoundedVoronoi):
    self.points = None
    self.BoundedVoronoi = BoundedVoronoi
    self.model = None
    self.temppoints = None
    self.newpoints = None
    self.pos = None

  def update_all(self,points,model):
      self.points = points
      self.temppoints = points
      self.newpoints = points
      self.model = model

  def getVertices(self,pos):
    return self.BoundedVoronoi.pointVertices(np.where((self.temppoints == pos).all(axis=1))[0][0])

  def sensorHat(self,pos): #Provides a sensor value at a (x,y) coordinate
    return self.model(pos[0],pos[1])

  def computeCentroid(self,vertices):
    xmin = 10000
    xmax = -10000
    ymin = 10000
    ymax = -10000
    for vertice in vertices:
      if(vertice[0]<xmin): # Tests for xmin
        xmin = vertice[0]
      if(vertice[0]>xmax): # Tests for xmax
        xmax = vertice[0]
      if(vertice[1]<ymin): # Tests for ymin
        ymin = vertice[1]
      if(vertice[1]>ymax): # Tests for ymax
        ymax = vertice[1]
    mass_moment = self.mass_moment(xmin,xmax,ymin,ymax)
    mass = mass_moment[0]
    moment = np.array([mass_moment[1],mass_moment[2]])
    centroid = np.divide(moment,mass)
    return centroid

  def mass_moment(self,xmin,xmax,ymin,ymax):
    massMoment,mMerror = mcquad(self.g23,npoints=1000,xl=[xmin,ymin],xu=[xmax,ymax],nprocs=1)
    return massMoment

  def g23(self,x_y):
    x23,y23 = x_y
    d = np.sqrt((self.pos[0]-x23)**2+(self.pos[1]-y23)**2)
    for point in self.temppoints:
      if d > np.sqrt((point[0]-x23)**2+(point[1]-y23)**2):
        return np.zeros((3,))
    value = self.sensorHat(x_y)
    if(value<.001):
      value = .001
    xyvalue = x_y*value
    return np.array([value,xyvalue[0],xyvalue[1]])

  def run(self):
    self.temppoints = self.points
    self.BoundedVoronoi.updateVoronoi(self.temppoints)
    count = 0
    while count < 20:
      for i,point in enumerate(self.temppoints):
        self.pos = point
        vertices = self.getVertices(self.pos)
        self.newpoints[i] = self.computeCentroid(vertices)
      self.temppoints = self.newpoints
      self.BoundedVoronoi.updateVoronoi(self.temppoints)
      count = count +1
    return self.temppoints
