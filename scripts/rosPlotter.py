#!/usr/bin/python
import rospy
import std_msgs
import numpy as np
from std_msgs.msg import String,Float64
from sensor_dist_ros.msg import floatArray as floatArray
from sensor_dist_ros.msg import sens as Data
from Sim import Sim
from BoundedVoronoi import BoundedVoronoi
from Plotter import PlotData
from Sensor import Sensor

global bigS,bounding_box,simVoronoi,sim,points,BigC
bounding_box = np.array([0,10,0,10])
soft_bounding_box = np.array([0,10,0,10])
VoronoiPlot = BoundedVoronoi(None,bounding_box)
plotter = PlotData(VoronoiPlot,Sensor(),soft_bounding_box)
count = 1

def robot0Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[0,:] = np.array(([data.x,data.y]))
def robot1Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[1,:] = np.array(([data.x,data.y]))
def robot2Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[2,:] = np.array(([data.x,data.y]))
def robot3Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[3,:] = np.array(([data.x,data.y]))
def modelCallback(data):
    global plotter
    plotter.linModel = data.floats

def initialize():
    rospy.init_node('rosCloud', anonymous = True)
    global points, plotter, count
    # The combined sensor reading and position of the reading in one message from each robot
    rospy.Subscriber("/robot0/data",Data,robot0Callback,queue_size=1)
    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
    # rospy.Subscriber("/edge/model",floatArray,modelCallback,queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if count%10 == 0:
            print count,'\n'
        if count%200 == 0:
            plotter.BoundedVoronoi.updateVoronoi(plotter.points)
            plotter.plotResults()
            count = count +1
        else:
            plotter.updatePath()
            count = count+1

        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
