#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import socket
import random

rospy.init_node('TWTOFV4Client', anonymous=True)

tx_pub = rospy.Publisher('/twtof/poll', Float32MultiArray, queue_size=10)

def send_message_to(whom, data):
    msg = Float32MultiArray(data=data)
    msg.layout.dim.append(MultiArrayDimension(label=whom))
    tx_pub.publish(msg)

resp_data = []
def sub_callback(msg):
    # print(f'Get {msg.data} from {msg.layout.dim[0].label}')
    global resp_data
    if msg.layout.dim[0].label == 'client':
        resp_data = list(msg.data)

rx_sub = rospy.Subscriber('/twtof/request', Float32MultiArray, sub_callback)


def test_once():
    tapoll = time.time()
    poll_data = float(random.randint(0, 1000))
    send_message_to('server', [poll_data] + [random.uniform(0, 1) for _ in range(10)])
    # print(f'Send {[poll_data]} to {other_name} @ {tapoll}')
    time_exceed_flag = False
    time_limit = 3.0
    global resp_data
    while len(resp_data) == 0 or resp_data[0] != poll_data:
        if time.time() > tapoll + time_limit:
            time_exceed_flag = True
            break
    if time_exceed_flag:
        print('Time Exceeded')
        return -1
    # print(f'Get response {resp_data}')
    taresp = time.time()
    treply = resp_data[-1]
    tround = taresp - tapoll
    return (tround - treply) / 2


class MY_STAT:
    def __init__(self):
        self.data = []
    
    def add_data(self, new_data):
        self.data.append(new_data)
        print('Test #{} tprop={:.3f}ms'.format(len(self.data), new_data * 1000))
    
    def show_res(self):
        print('------- Client test statistics -------')
        print('tprop min/avg/max = {:.3f}/{:.3f}/{:.3f} ms'.format(
            min(self.data) * 1000,
            sum(self.data)/len(self.data) * 1000,
            max(self.data) * 1000))

time.sleep(1)

my_stat = MY_STAT()
num_input = input('Input test number (Default: 10): ')
num = 10 if num_input == '' else int(num_input)
for _ in range(num):
    res = test_once()
    if res > 0:
        my_stat.add_data(res)
my_stat.show_res()
