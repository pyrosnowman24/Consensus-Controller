#!/usr/bin/python
import numpy as np
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
from skmonaco import mcquad, mcmiser
import turtlesim.srv

class End:
  def __init__(self,index,pos,Sensor,startTime,bounding_box):
    self.index = index
    self.pos = pos
    self.centroid = 0
    self.points = np.zeros((4,2))
    self.p = np.array([1,1,1])
    self.Sensor = Sensor
    self.startTime = startTime
    self.prevTime = startTime
    self.K = 1
    self.prevControl = 0
    self.currentMass = 0
    self.eps = sys.float_info.epsilon
    self.centroids = 0
    self.bounding_box = bounding_box
    self.vor = self.voronoi(self.points, bounding_box)

  def sensorHat(self,pos = None): #Provides a sensor value at a (x,y) coordinate
    if pos is None:
      pos = self.pos
    sum = self.p[2]+(self.p[0]*pos[0]+self.p[1]*pos[1])
    if sum == 0:
        sum = np.array(.0001)
    return sum

  def computeCentroid(self):
    xmin = 10000
    xmax = -10000
    ymin = 10000
    ymax = -10000
    for vertice in self.pointVertices():
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
    self.currentMass = mass
    moment = np.array([mass_moment[1],mass_moment[2]])
    centroid = np.divide(moment,mass)
    self.centroid = centroid
    return centroid

  def mass_moment(self,xmin,xmax,ymin,ymax):
    massMoment,mMerror = mcquad(self.g23,npoints=1000,xl=[xmin,ymin],xu=[xmax,ymax],nprocs=1)
    return massMoment

  def g23(self,x_y):
    x23,y23 = x_y
    d = np.sqrt((self.pos[0]-x23)**2+(self.pos[1]-y23)**2)
    for point in self.points:
      if d > np.sqrt((point[0]-x23)**2+(point[1]-y23)**2):
        return np.zeros((3,))
    value = self.sensorHat(x_y)
    if(value<.001):
      value = .001
    xyvalue = x_y*value
    return np.array([value,xyvalue[0],xyvalue[1]])

  def updateVoronoi(self):
    self.vor = self.voronoi(self.points,self.bounding_box)

  def pointVertices(self):
    return self.vor.vertices[self.vor.regions[self.vor.point_region[self.index]],:]

  def in_box(self,towers, bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],towers[:, 0] <= bounding_box[1]),np.logical_and(bounding_box[2] <= towers[:, 1],towers[:, 1] <= bounding_box[3]))

  def voronoi(self, towers, bounding_box):
      self.points = towers
      # Select towers inside the bounding box
      i = self.in_box(towers, bounding_box)
      # Mirror points
      points_center = towers[i, :]
      points_left = np.copy(points_center)
      points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
      points_right = np.copy(points_center)
      points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
      points_down = np.copy(points_center)
      points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
      points_up = np.copy(points_center)
      points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
      points = np.append(points_center,np.append(np.append(points_left,points_right,axis=0),np.append(points_down,points_up,axis=0),axis=0),axis=0)
      # Compute Voronoi
      vor = Voronoi(points)
      # Filter regions
      regions = []
      for region in vor.regions:
          flag = True
          for index in region:
              if index == -1:
                  flag = False
                  break
              else:
                  x = vor.vertices[index, 0]
                  y = vor.vertices[index, 1]
                  if not(bounding_box[0] - self.eps <= x and x <= bounding_box[1] + self.eps and
                         bounding_box[2] - self.eps <= y and y <= bounding_box[3] + self.eps):
                      flag = False
                      break
          if region != [] and flag:
              regions.append(region)
      vor.filtered_points = points_center
      vor.filtered_regions = regions
      return vor
