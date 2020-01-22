#!/usr/bin/python
import numpy as np
import math
import random
from scipy import optimize
import rospy
import tf_conversions
import tf2_ros
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist
from gazebo_msgs.msg import ModelStates
from sensor_dist_ros.msg import sens as Data
from sensor_dist_ros.msg import floatArray as floatArray
from sensor_dist_ros.msg import floatArray2 as floatArray2
import turtlesim
from End import End
from Sensor import Sensor

rospy.init_node('rosEnd', anonymous = True)

global sensor_function,bounding_box,desired_position,robot,current_message,first_cloud_recieved,turtle_vel,id,tfBuffer
id = rospy.get_param('~id')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
current_message = None
sensor_function = Sensor()
bounding_box = np.array((0,10,0,10))
desired_position = [np.array([float('nan'),float('nan')])]
robot = End(id,[random.uniform(bounding_box[0],bounding_box[1]),random.uniform(bounding_box[2],bounding_box[3])],sensor_function,rospy.get_time(),bounding_box)
first_cloud_recieved = False

def posesCallback(data):
    global desired_position,first_cloud_recieved
    desired_position = np.array([data.x[id],data.y[id]])
    if not np.isnan(np.sum(desired_position)):
        print desired_position
        first_cloud_recieved = True
    # print "desired",desired_position[:,0]
def modelCallback(data):
    if data.floats is None:
        return None
    else:
        robot.p = data.floats
def handle_turtle_pose(msg):
    global current_message
    if len(msg.name) == 5:
        current_message = msg
def robot0Callback(data):
    robot.points[0,:] = [data.x,data.y]
def robot1Callback(data):
    robot.points[1,:] = [data.x,data.y]
def robot2Callback(data):
    robot.points[2,:] = [data.x,data.y]
def robot3Callback(data):
    robot.points[3,:] = [data.x,data.y]

def broadcast_locations(): # Responisble for publishing the transform between each robot and "world"
    global current_message
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "robot"+str(id)+"/pos"
    name = "robot"+str(id)
    # Finds position and sets transform to world
    index = current_message.name.index(name)
    pose = current_message.pose[index]
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    # print "transform_pos_x=",pose.position.x,"transform_pos_y=",pose.position.y,"transform_pos_z=",pose.position.z
    br.sendTransform(t)
def broadcast_desired_pos():
    global desired_position
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now
    # Publishes transform for the desired position
    # print msg.pose.pose, '\r'
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "robot"+str(id)+"/desired_position"
    t.transform.translation.x = desired_position[0]
    t.transform.translation.y = desired_position[1]
    # print "transform_desired_x=",desired_position[0],"transform_desired_y=",desired_position[1]
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)
def generate_control():
    global turtle_vel
    try:
        trans = tfBuffer.lookup_transform("robot"+str(id)+"/pos","robot"+str(id)+"/desired_position",rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print "LookupException"
        return None
    msg = Twist()
    # print trans.transform.translation,'\r'
    # print math.sqrt(trans.transform.translation.y**2+trans.transform.translation.x**2)
    if math.sqrt(trans.transform.translation.y**2+trans.transform.translation.x**2) < .3:
        msg.angular.z = 0
        msg.linear.x = 0
    else:
        msg.angular.z = 2 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.25 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
    # print "linear: ",msg.linear.x,"angular: ",msg.angular.z
    turtle_vel.publish(msg)
def get_positions(all_ids):
    global robot,tfBuffer
    for member in all_ids:
        try:
            position_transform = tfBuffer.lookup_transform("world","robot"+str(member)+"/pos",rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "LookupException"
            return True
        robot.points[member,:] = [position_transform.transform.translation.x,position_transform.transform.translation.y]
        if member == id:
            robot.pos = [position_transform.transform.translation.x,position_transform.transform.translation.y]
    return False
def initialize():
    all_ids = [0,1,2,3]
    others = [0,1,2,3]
    others.remove(id)
    global desired_position,robot,turtle_vel
    rospy.Subscriber("/cloud/poses",floatArray2,posesCallback,queue_size=1)
    rospy.Subscriber("/edge/model",floatArray,modelCallback,queue_size=1)
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_turtle_pose)

    turtle_vel = rospy.Publisher('/robot%s/mobile_base/commands/velocity' % id, Twist, queue_size=1)
    data = rospy.Publisher("/robot%s/data"%id,Data,queue_size=1,latch = True)
    rate = rospy.Rate(10)
    # data.publish(robot.pos[0],robot.pos[1],robot.Sensor.sensor(robot.pos))

    while not rospy.is_shutdown():
        if current_message is not None: # if we have recieved a message then update the transforms
            broadcast_locations()
        if get_positions(all_ids): # If the transforms return an error, then hold until they are ready
            print "Sleeping"
            rate.sleep()
        else:
            if np.count_nonzero(robot.points) != robot.points.size: # Check if a position was recieved from each robot
                print "zeros!"
            if not first_cloud_recieved: # Runs if the cloud hasnt sent a desired position yet
                print "Speed Layer"
                robot.updateVoronoi()
                centroid = robot.computeCentroid()
                desired_position = centroid
            else: # Runs if the cloud has sent a desired position
                print "Batch Layer"
                print "Position:",robot.pos
                print "Desired:",desired_position
            if not np.isnan(np.sum(desired_position)):
                broadcast_desired_pos()
            if not np.isnan(np.sum(desired_position)):
                generate_control()
            data.publish(robot.pos[0],robot.pos[1],robot.Sensor.sensor(robot.pos))
        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
