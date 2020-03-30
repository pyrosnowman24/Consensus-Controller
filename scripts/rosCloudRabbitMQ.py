#!/usr/bin/python
import rospy
import numpy as np
import std_msgs
import pika
import yaml
import json
import signal
import time
from std_msgs.msg import String,Float64
from sensor_dist_ros.msg import floatArray as floatArray
from sensor_dist_ros.msg import floatArray2 as floatArray2
from sensor_dist_ros.msg import sens as Data
from Cloud import Cloud
from Sim import Sim
from BoundedVoronoi import BoundedVoronoi

global bigS,bounding_box,simVoronoi,sim,points,BigC,count
bounding_box = np.array([0,10,0,10])
soft_bounding_box = np.array([0,10,0,10])
simVoronoi = BoundedVoronoi(None,bounding_box)
sim = Sim(simVoronoi,bounding_box,soft_bounding_box)
points = np.zeros((4,2))
BigC = Cloud(sim)
count = 0

def robot0Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[0,:] = np.array(([data.x,data.y]))
def robot1Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[1,:] = np.array(([data.x,data.y]))
def robot2Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[2,:] = np.array(([data.x,data.y]))
def robot3Callback(data):
    global points,BigC
    BigC.updateData([data.x,data.y,data.sensor])
    points[3,:] = np.array(([data.x,data.y]))
# Functions for pika/RabbitMQ

def close_pika(signal, frame):
    print('Closing Pika Connection')
    connection.close()
    sys.exit(0)
def publish_to_mq(entry):
    assignment_to_send = json.dumps(entry)
    channel.basic_publish(exchange='cloud_model_info',routing_key='key_cloud_model_info',body=assignment_to_send)
    time.sleep(0.01)

if __name__ == '__main__':
    try:
   	rospy.init_node('rosCloud', anonymous = True)
	### Read config parameters for RabbitMQ
	signal.signal(signal.SIGTERM, close_pika)
	hostname = rospy.get_param('hostname')
	username = rospy.get_param('username')
	password = rospy.get_param('password')
	port = rospy.get_param('port')
	credentials = pika.PlainCredentials(username, password)
	connection = pika.BlockingConnection(pika.ConnectionParameters(host=hostname, credentials=credentials, port=port))
	channel = connection.channel()
	channel.exchange_declare(exchange='cloud_model_info', exchange_type='direct')
	# The combined sensor reading and position of the reading in one message from each robot
	rospy.Subscriber("/robot0/data",Data,robot0Callback,queue_size=1)
	rospy.Subscriber("/robot1/data",Data,robot1Callback,queue_size=1)
	rospy.Subscriber("/robot2/data",Data,robot2Callback,queue_size=1)
	rospy.Subscriber("/robot3/data",Data,robot3Callback,queue_size=1)
	startTime = rospy.get_time()
	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		global points, count
		for point in points: # Tests if any of the robots positions hasent been recieved yet
		    if point[0] == 0 or point[1] == 0:
		        print "~~~~~~~~~~~~PANIC~~~~~~~~~~~~~~~~"
		        rate.sleep()
		if(BigC.data is not None and len(BigC.data) >= 100):
		    print '\n',"Publishing model",count,'\n'
		    start_time = rospy.get_time()
		    BigC.updateModel()
		    runtime_file = open('/home/ubuntu/runtimes/cloud_modeling_runtime.csv','a')
	            np.savetxt(runtime_file,np.array(rospy.get_time()-start_time).reshape(1,),delimiter=',')
		    count = count + 1
		    sim_start = rospy.get_time()
		    poses = BigC.findPositions(points).transpose()
		    print "Simulation concluded",count,":",rospy.get_time() - sim_start
		    runtime_file = open('/home/ubuntu/runtimes/cloud_pos_runtime.csv','a')
	    	    np.savetxt(runtime_file,np.array(rospy.get_time()-sim_start).reshape(1,),delimiter=',')
		    message = [list(poses[0,:]),list(poses[1,:])]
		    publish_to_mq(message)
		else:
		    print "Publishing 0",'\n'
		    if(BigC.data is not None):
		        print "Data Collected:", len(BigC.data)
		rate.sleep()
    except rospy.ROSInterruptException:
        pass

