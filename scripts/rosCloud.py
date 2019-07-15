import rospy
import std_msgs
from std_msgs.msg import String,Float64
from Sensor_Dist_ROS.msg import floatArray as floatArray
from Sensor_Dist_ROS.msg import floatArray2 as floatArray2
from Sensor_Dist_ROS.msg import sens as Sens
import turtlesim
from Cloud import Cloud

BigC = Cloud()

def robot1Callback(data):
    data.publish(data)
    BigC.updateData([data.x,data.y,data.sensor])
def robot2Callback(data):
    data.publish(data)
    BigC.updateData([data.x,data.y,data.sensor])
def robot3Callback(data):
    data.publish(data)
    BigC.updateData([data.x,data.y,data.sensor])
def robot4Callback(data):
    data.publish(data)
    BigC.updateData([data.x,data.y,data.sensor])

def initialize():
    rospy.init_node('rosCloud', anonymous = True)
    # The combined sensor reading and position of the reading in one message from each robot
    rospy.Subscriber("/robot1/data",Sens,dataCallback,queue_size=1)
    rospy.Subscriber("/robot2/data",Sens,dataCallback,queue_size=1)
    rospy.Subscriber("/robot3/data",Sens,dataCallback,queue_size=1)
    rospy.Subscriber("/robot4/data",Sens,dataCallback,queue_size=1)
    # The published array of robot positions and the array of values for the model
    poses = rospy.Publisher("/cloud/poses",floatArray2,queue_size=1)

    rate = rospy.Rate(80)

    while not rospy.is_shutdown():
        BigC.updateModel()
        Poses.publish(BigC.poses[:,0],BigC.poses[:,1])
        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
