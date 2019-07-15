import numpy as np
import random
from scipy import optimize
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseArray
from Sensor_Dist_ROS.msg import sens as Data
from Sensor_Dist_ROS.msg import floatArray as floatArray
import turtlesim
from End import End
from Sensor import Sensor

rospy.init_node('rosEnd1', anonymous = True)
bigS = Sensor()
bounding_box = np.array((0,1,0,1))
robot = End(3,[.75,.75],bigS,rospy.get_time(),bounding_box)
def posesCallback(data):
    robot.pos = data[:,4]
def modelCallback(data):
    robot.p = data
def robot1Callback(data):
    router.positions[1,:] = [data.x,data.y]
def robot2Callback(data):
    router.positions[2,:] = [data.x,data.y]
def robot3Callback(data):
    router.positions[3,:] = [data.x,data.y]
# def localizationCallback(data): # Callback for localization data
#     robot.pos = data

def initialize():

    rospy.Subscriber("/cloud/poses",floatArray2,posesCallback,queue_size=1)
    rospy.Subscriber("/edge/model",floatArray,modelCallback,queue_size=1)
    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
    # rospy.Subscriber("/robot/localization",Float64[],loalizationCallback,queue_size=1) # The subscription to the ouput localication value goes here
    data = rospy.Publisher("/robot4/data",Data,queue_size=1)
    rate = rospy.Rate(20)

    data.publish(robot.pos[0],robot.pos[1],robot.Sensor.sensor(robot.pos))
    
    while not rospy.is_shutdown():
        robot.positions[4,:] = robot.pos
        robot.updateVoronoi()
        centroid = robot.computeCentroid()
        robot.updatePosition(centroid)
        data.publish(robot.pos[0],robot.pos[1],robot.Sensor.sensor(robot.pos))
        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
