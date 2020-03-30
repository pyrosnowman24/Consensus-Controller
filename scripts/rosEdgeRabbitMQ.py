#!/usr/bin/python
import rospy
import std_msgs
import pika
import yaml
import json
import signal
import time
import sys
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray,Pose
from sensor_dist_ros.msg import sens as Data
from sensor_dist_ros.msg import floatArray as floatArray
from Edge import Edge

global router
router = Edge()
# Callbacks for ROS
def robot0Callback(data):
    print(sys.getsizeof(data))
    router.data[0,:]=[data.x,data.y,data.sensor]
def robot1Callback(data):
    router.data[1,:]=[data.x,data.y,data.sensor]
def robot2Callback(data):
    router.data[2,:]=[data.x,data.y,data.sensor]
def robot3Callback(data):
    router.data[3,:]=[data.x,data.y,data.sensor]
# Functions for pika/RabbitMQ
def close_pika(signal, frame):
    print('Closing Pika Connection')
    connection.close()
    sys.exit(0)
def publish_to_mq(entry):
    assignment_to_send = json.dumps(entry)
    channel.basic_publish(exchange='robot_pos_info',routing_key='key_robot_pos_info',body=assignment_to_send)
    time.sleep(0.01)

#def initialize():


if __name__ == '__main__':
    try:
            rospy.init_node("rosEdge", anonymous = True)
	    ### Read config parameters for RabbitMQ
	    signal.signal(signal.SIGTERM, close_pika)
	    hostname = rospy.get_param('hostname')
	    username = rospy.get_param('username')
	    password = rospy.get_param('password')
	    port = rospy.get_param('port')
	    credentials = pika.PlainCredentials(username, password)
	    connection = pika.BlockingConnection(pika.ConnectionParameters(host=hostname, credentials=credentials, port=port))
	    channel = connection.channel()
	    channel.exchange_declare(exchange='robot_pos_info', exchange_type='direct')
	    # The combined sensor reading and position of the reading in one message from each robot
	    rospy.Subscriber("/robot0/data",Data,robot0Callback,queue_size=1)
	    rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
	    rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
	    rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
	    # The published array of robot positions and the array of values for the model
	    model = rospy.Publisher("/edge/model",floatArray,queue_size=1)
	    rate = rospy.Rate(10)
	    # Main Loop
	    while not rospy.is_shutdown():
		start_time = rospy.get_time()
		router.linearApprox()		
		runtime_file = open('/home/ace/runtimes/edge_runtime.csv','a')
		np.savetxt(runtime_file,np.array(rospy.get_time()-start_time).reshape(1,),delimiter=',')
		model.publish(router.edgeP)
		if np.sum(router.data[0,:])!=0:
			publish_to_mq([0,router.data[0,0],router.data[0,1],router.data[0,2]])
		if np.sum(router.data[1,:])!=0:
			publish_to_mq([1,router.data[1,0],router.data[1,1],router.data[1,2]])
		if np.sum(router.data[2,:])!=0:
			publish_to_mq([2,router.data[2,0],router.data[2,1],router.data[2,2]])
		if np.sum(router.data[3,:])!=0:
			publish_to_mq([3,router.data[3,0],router.data[3,1],router.data[3,2]])
		rate.sleep()
    except rospy.ROSInterruptException:
        pass
