#!/usr/bin/env python3

from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import rospy

class BoundedVoronoi:
  def __init__(self,points,bounding_box):
    self.eps = sys.float_info.epsilon
    self.points = points
    self.centroids = 0
    self.bounding_box = bounding_box
    self.vor = self.voronoi(points, bounding_box)
    rospy.init_node('Voronoi_Node')
  def updateVoronoi(self,points,bounding_box = None):
    if bounding_box is None:
      bounding_box = self.bounding_box
    self.vor = self.voronoi(points,bounding_box)
  def pointVertices(self,index):
    return self.vor.vertices[self.vor.regions[self.vor.point_region[index]],:]
  def in_box(self,towers, bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],towers[:, 0] <= bounding_box[1]),np.logical_and(bounding_box[2] <= towers[:, 1],towers[:, 1] <= bounding_box[3]))

  def voronoi(self, towers, bounding_box = None):
      if bounding_box is None:
        bounding_box = self.bounding_box
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

  def centroid_region(self,vertices):
      # Polygon's signed area
      A = 0
      # Centroid's x
      C_x = 0
      # Centroid's y
      C_y = 0
      for i in range(0, len(vertices) - 1):
          s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
          A = A + s
          C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
          C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
      A = 0.5 * A
      C_x = (1.0 / (6.0 * A)) * C_x
      C_y = (1.0 / (6.0 * A)) * C_y
      return np.array([[C_x, C_y]])
  def plot_voronoi(self):
      fig = plt.figure()
      ax = fig.gca()
      # Plot initial points
      ax.plot(self.vor.filtered_points[:, 0], self.vor.filtered_points[:, 1], 'b.')
      # Plot ridges points
      for region in self.vor.filtered_regions:
          vertices = self.vor.vertices[region, :]
          ax.plot(vertices[:, 0], vertices[:, 1], 'go')
      # Plot ridges
      for region in self.vor.filtered_regions:
          vertices = self.vor.vertices[region + [region[0]], :]
          ax.plot(vertices[:, 0], vertices[:, 1], 'k-')
      for centroid in self.centroids:
          ax.plot(centroid[0],centroid[1],'bx')
#       Compute and plot centroids
#       centroids = []
#       for region in self.vor.filtered_regions:
#           vertices = self.vor.vertices[region + [region[0]], :]
#           centroid = self.centroid_region(vertices)
#           centroids.append(list(centroid[0, :]))
#           ax.plot(centroid[:, 0], centroid[:, 1], 'r.')
      ax.set_xlim([-0.1, 1.1])
      ax.set_ylim([-0.1, 1.1])
      plt.show()
