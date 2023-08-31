#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import socket
import random

rospy.init_node('TWTOFV4Server', anonymous=True)

tx_pub = rospy.Publisher('/twtof/request', Float32MultiArray, queue_size=10)

def send_message_to(whom, data):
    msg = Float32MultiArray(data=data)
    msg.layout.dim.append(MultiArrayDimension(label=whom))
    tx_pub.publish(msg)

poll_data = []
def sub_callback(msg):
    if msg.layout.dim[0].label == 'server':
        tbpoll = time.time()
        poll_data = list(msg.data)
        # print(f'Get poll {poll_data} from {other_name}')
        treply = random.uniform(0.5, 1.5)
        poll_data.append(treply)
        time.sleep(treply)
        # print(f'After {treply}, send back response to {other_name}')
        send_message_to('client', poll_data)

rx_sub = rospy.Subscriber('/twtof/poll', Float32MultiArray, sub_callback)

test_res = []


print('Start Server...')
rospy.spin()