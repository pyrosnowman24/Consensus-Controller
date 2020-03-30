#!/usr/bin/env python
import pika
import yaml
import json
import signal
import time
import rospy
from sensor_dist_ros.msg import floatArray as floatArray2
from sensor_dist_ros.msg import sens as Data
global r0,r1,r2,r3
rospy.init_node('rosCloudReciever',anonymous=True)
r0 = rospy.Publisher("/robot0/data",Data,queue_size=1)
r1 = rospy.Publisher("/robot1/data",Data,queue_size=1)
r2 = rospy.Publisher("/robot2/data",Data,queue_size=1)
r3 = rospy.Publisher("/robot3/data",Data,queue_size=1)
rate = rospy.Rate(10)
def close_pika(signal, frame):
    print('Closing Pika Connection')
    connection.close()
    sys.exit(0)
def callback(ch, method, properties, body):
    global r0,r1,r2,r3
    info = json.loads(body.decode('utf-8'))
    print(info)
    if info[0] == 0:
        r0.publish(info[1],info[2],info[3])
    if info[0] == 1:
        r1.publish(info[1],info[2],info[3])
    if info[0] == 2:
        r2.publish(info[1],info[2],info[3])
    if info[0] == 3:
        r3.publish(info[1],info[2],info[3])
def initialize():
    global r0, r1, r2, r3
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
    results = channel.queue_declare(queue="",exclusive=True)
    queue_name = results.method.queue
    channel.queue_bind(exchange='robot_pos_info',queue=queue_name,routing_key='key_robot_pos_info')
    channel.basic_consume(on_message_callback=callback,queue=queue_name,auto_ack=True)
    channel.start_consuming()
    # Main Loop
    while not rospy.is_shutdown():
        rate.sleep()
if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
