#!/usr/bin/python
import rospy
import std_msgs
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray,Pose
from sensor_dist_ros.msg import sens as Data
from sensor_dist_ros.msg import floatArray as floatArray
from Edge import Edge

global router
router = Edge()

def robot0Callback(data):
    router.data[0,:]=[data.x,data.y,data.sensor]
def robot1Callback(data):
    router.data[1,:]=[data.x,data.y,data.sensor]
def robot2Callback(data):
    router.data[2,:]=[data.x,data.y,data.sensor]
def robot3Callback(data):
    router.data[3,:]=[data.x,data.y,data.sensor]

def initialize():
    rospy.init_node('rosEdge', anonymous = True)
    # The combined sensor reading and position of the reading in one message from each robot
    rospy.Subscriber("/robot0/data",Data,robot0Callback,queue_size=1)
    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
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
