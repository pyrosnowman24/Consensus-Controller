#!/usr/bin/python
import numpy as np
import rospy
from scipy.spatial import Voronoi, voronoi_plot_2d
import sys
from skmonaco import mcquad, mcmiser

class Sim:
  def __init__(self,BoundedVoronoi,bounding_box,soft_bounding_box):
    self.points = None
    self.BoundedVoronoi = BoundedVoronoi
    self.model = None
    self.temppoints = None
    self.pos = None
    self.center = np.zeros((1,2))
    self.soft_bounding_box = soft_bounding_box
    self.bounding_box = bounding_box

  def update_all(self,points,model):
      self.points = points
      self.temppoints = points
      self.model = model

  def getVertices(self,pos):
    return self.BoundedVoronoi.pointVertices(np.where((self.temppoints == pos).all(axis=1))[0][0])

  def sensorHat(self,pos): #Provides a sensor value at a (x,y) coordinate
    return self.model(pos[0],pos[1])**5

  def sensor_Pre_Processing(self,sensor_value,pos): # Adjusts the sensor values to add a cost to being outide the desired region
    scale = lambda x1,x2 : 20/(x2-x1+.001) # .001 prevents undefined at x2-x1 = 0
    cost = lambda pos,s,bound : 1/(1+np.exp(s*pos-4-s*bound)) # Calculates cost for position
    if (pos[0]<self.soft_bounding_box[0]):
      # Position is outside negative x of box
      s_x = scale(self.soft_bounding_box[0],self.bounding_box[0])
      x_bound = self.soft_bounding_box[0]
      if self.soft_bounding_box[0] == self.bounding_box[0]:
        s_x = 0
    elif (pos[0]>self.soft_bounding_box[1]):
      # Position is outisde positive part of box
      s_x = scale(self.soft_bounding_box[1],self.bounding_box[1])
      x_bound = self.soft_bounding_box[1]
      if self.soft_bounding_box[1] == self.bounding_box[1]:
        s_x = 0
    else:
      s_x = 0
      x_bound = 0

    if (pos[1]<self.soft_bounding_box[2]):
      # Position is outide negative y of box
      s_y = scale(self.soft_bounding_box[2],self.bounding_box[2])
      y_bound = self.soft_bounding_box[2]
      if self.soft_bounding_box[2] == self.bounding_box[2]:
        s_y = 0
    elif (pos[1]>self.soft_bounding_box[3]):
      # Position is outisde positive y of box
      s_y = scale(self.soft_bounding_box[3],self.bounding_box[3])
      y_bound = self.soft_bounding_box[3]
      if self.soft_bounding_box[3] == self.bounding_box[3]:
        s_y = 0
    else:
      # Position is inside box
      s_y = 0
      y_bound = 0
    #     print("pos_x:",pos[0],"s_x:",s_x,"x_bound:",x_bound,"Cost_x:",cost(pos[0],s_x,x_bound))
    #     print("pos_y:",pos[1],"s_y:",s_y,"y_bound:",y_bound,"Cost_y:",cost(pos[1],s_y,y_bound))
    #     print("cost_x:",cost(pos[0],s_x,x_bound),"cost_y:",cost(pos[1],s_y,y_bound),"combined cost:",cost(pos[0],s_x,x_bound)*cost(pos[1],s_y,y_bound))
    return sensor_value *cost(pos[1],s_y,y_bound)*cost(pos[0],s_x,x_bound)

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
    value = self.sensor_Pre_Processing(self.sensorHat(x_y),x_y)
    if(value<.001):
      value = .001
    xyvalue = x_y*value
    return np.array([value,xyvalue[0],xyvalue[1]])

  def run(self):
    startTime = rospy.get_time()
    print "Starting simulation"
    self.temppoints = self.points
    newpoints = np.zeros((4,2))
    self.BoundedVoronoi.updateVoronoi(self.temppoints)
    count = 0
    amount = 25
    while count < amount:
      y_sum = 0
      x_sum = 0
      for point in self.temppoints:
          x_sum += point[0]
          y_sum += point[1]
      self.center = np.array([x_sum/self.temppoints.shape[0],y_sum/self.temppoints.shape[0]])
      # print "Iteration",count,"ellapsed time",rospy.get_time() - startTime
      # print "Curent points:",'\n',self.temppoints
      for i,point in enumerate(self.temppoints):
        self.pos = point
        vertices = self.getVertices(self.pos)
        centroid = self.computeCentroid(vertices)
        adjusted_desired_position = centroid + .1 * np.subtract(self.center,centroid)
        # adjusted_desired_position = centroid
        newpoints[i] = point + .5 * np.subtract(adjusted_desired_position,point)
      if np.less(np.absolute(np.subtract(self.temppoints,newpoints)),.2).all(): # the .2 is used to determine the threshold for equilibrium
          self.temppoints = newpoints+0 # zero forces a new memory value for self.tempoints, otherwise itll change values with newpoints
          count = amount
      else:
          self.temppoints = newpoints+0
          count = count +1
      self.BoundedVoronoi.updateVoronoi(self.temppoints)
    return self.temppoints
