#!/usr/bin/env python
import pika
import yaml
import json
import signal
import time
import rospy
from sensor_dist_ros.msg import floatArray2 as floatArray2

global desired_pose_data,desiredPoses

def close_pika(signal, frame):
    print('Closing Pika Connection')
    connection.close()
    sys.exit(0)

def callback(ch, method, properties, body):
    global desired_pose_data,desiredPoses
    info = json.loads(body.decode('utf-8'))
    print(info)
    desired_pose_data = info
    desiredPoses.publish(list(info[0]),list(info[1]))
    print("published")

def initialize():
    ### Read config parameters for RabbitMQ
    global desired_pose_data,desiredPoses
    desired_pose_data = None
    rospy.init_node('rosEdge', anonymous = True)
    rate = rospy.Rate(10)
    desiredPoses = rospy.Publisher("/cloud/poses",floatArray2,queue_size=1)
    signal.signal(signal.SIGTERM, close_pika)
    hostname = rospy.get_param('hostname')
    username = rospy.get_param('username')
    password = rospy.get_param('password')
    port = rospy.get_param('port')
    credentials = pika.PlainCredentials(username, password)
    connection = pika.BlockingConnection(pika.ConnectionParameters(host=hostname, credentials=credentials, port=port))
    channel = connection.channel()
    channel.exchange_declare(exchange='cloud_model_info', exchange_type='direct')
    results = channel.queue_declare(queue="",exclusive=True)
    queue_name = results.method.queue
    channel.queue_bind(exchange='cloud_model_info',queue=queue_name,routing_key='key_cloud_model_info')
    channel.basic_consume(on_message_callback=callback,queue=queue_name)
    channel.start_consuming()
    # The published array of robot positions and the array of values for the model

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
