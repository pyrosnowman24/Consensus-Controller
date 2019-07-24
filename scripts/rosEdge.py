#!/usr/bin/python
import rospy
import std_msgs
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray,Pose
from Sensor_Dist_ROS.msg import sens as Data
from Sensor_Dist_ROS.msg import floatArray as floatArray
import turtlesim
from Edge import Edge

global router
router = Edge()

def robot1Callback(data):
    router.updateData([data.x,data.y,data.sensor])
    router.positions[0,:] = [data.x,data.y]
def robot2Callback(data):
    router.updateData([data.x,data.y,data.sensor])
    router.positions[1,:] = [data.x,data.y]
def robot3Callback(data):
    router.updateData([data.x,data.y,data.sensor])
    router.positions[2,:] = [data.x,data.y]
def robot4Callback(data):
    router.updateData([data.x,data.y,data.sensor])
    router.positions[3,:] = [data.x,data.y]

def initialize():
    rospy.init_node('rosEdge', anonymous = True)
    # The combined sensor reading and position of the reading in one message from each robot
    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
    rospy.Subscriber("/robot4/data",Data,robot4Callback,queue_size=1)
    # The published array of robot positions and the array of values for the model
    model = rospy.Publisher("/edge/model",floatArray,queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        router.linearApprox()
        model.publish(router.edgeP)
        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass