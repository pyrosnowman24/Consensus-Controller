#!/usr/bin/python
import rospy
import std_msgs
import numpy as np
from std_msgs.msg import String,Float64
from Sensor_Dist_ROS.msg import floatArray as floatArray
from Sensor_Dist_ROS.msg import sens as Data
from Sim import Sim
from BoundedVoronoi import BoundedVoronoi
from Plotter import PlotData
from Sensor import Sensor

global bigS,bounding_box,simVoronoi,sim,points,BigC
bounding_box = np.array([0,1,0,1])
VoronoiPlot = BoundedVoronoi(None,bounding_box)
plotter = PlotData(VoronoiPlot,Sensor())
count = 1

def robot1Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[0,:] = np.array(([data.x,data.y]))
def robot2Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[1,:] = np.array(([data.x,data.y]))
def robot3Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[2,:] = np.array(([data.x,data.y]))
def robot4Callback(data):
    global plotter
    plotter.updateData([data.x,data.y,data.sensor])
    plotter.points[3,:] = np.array(([data.x,data.y]))
def modelCallback(data):
    global plotter
    plotter.linModel = data.p

def initialize():
    rospy.init_node('rosCloud', anonymous = True)
    global points, plotter, count
    # The combined sensor reading and position of the reading in one message from each robot
    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
    rospy.Subscriber("/robot4/data",Data,robot4Callback,queue_size=1)
    rospy.Subscriber("/edge/model",floatArray,modelCallback,queue_size=1)

    rate = rospy.Rate(5)

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
