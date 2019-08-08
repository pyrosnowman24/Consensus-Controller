#!/usr/bin/python
import rospy
import numpy as np
import std_msgs
from std_msgs.msg import String,Float64
from Sensor_Dist_ROS.msg import floatArray as floatArray
from Sensor_Dist_ROS.msg import floatArray2 as floatArray2
from Sensor_Dist_ROS.msg import sens as Data
import turtlesim
from Cloud import Cloud
from Sim import Sim
from BoundedVoronoi import BoundedVoronoi

global bigS,bounding_box,simVoronoi,sim,points,BigC
bounding_box = np.array([0,1,0,1])
simVoronoi = BoundedVoronoi(None,bounding_box)
sim = Sim(simVoronoi)
points = np.zeros((4,2))
BigC = Cloud(sim)
count = 0

def robot1Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[0,:] = np.array(([data.x,data.y]))
def robot2Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[1,:] = np.array(([data.x,data.y]))
def robot3Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[2,:] = np.array(([data.x,data.y]))
def robot4Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[3,:] = np.array(([data.x,data.y]))

def initialize():
    rospy.init_node('rosCloud', anonymous = True)
    # The combined sensor reading and position of the reading in one message from each robot
    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
    rospy.Subscriber("/robot4/data",Data,robot4Callback,queue_size=1)
    # The published array of robot positions and the array of values for the model
    desiredPoses = rospy.Publisher("/cloud/poses",floatArray2,queue_size=1)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        global points, count
        # print points
        # if BigC.data is not None:
        #     print len(BigC.data)
        for point in points: # Tests if any of the robots positions hasent been recieved yet
            if point[0] == 0 or point[1] == 0:
                print "~~~~~~~~~~~~PANIC~~~~~~~~~~~~~~~~"
                rate.sleep()
        if(BigC.data is not None and len(BigC.data) >= 100):
            BigC.updateModel()
            count = count + 1
            poses = BigC.findPositions(points).transpose()
            # print "Cloud Model:",'\n'
            # print "x=.2,y=.2,z=",BigC.model(.2,.2)
            # print "x=.75,y=.32,z=",BigC.model(.75,.32)
            # print "x=.46,y=.78,z=",BigC.model(.46,.78)
            # print "x=.45,y=.26,z=",BigC.model(.45,.26)
            # print "x=.16,y=.52,z=",BigC.model(.16,.52)
            print "Publishing model",count,'\n'
            desiredPoses.publish(list(poses[0,:]),list(poses[1,:]))
        else:
            print "Publishing 0",'\n'
            desiredPoses.publish([float('nan')],[float('nan')])
        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
