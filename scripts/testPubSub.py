#! /usr/bin/env python

import rospy
import roslib.message

if __name__ == '__main__':
    rospy.init_node('testPubSub', anonymous=True)

    namespace = 'testPubSub/'
    vehicleName = rospy.get_param(namespace + 'vehicle_name', {})
    pub_topics = rospy.get_param(namespace + 'send_topics', {})
    sub_topics = rospy.get_param(namespace + 'recv_topics', {})

#######pub#########

    print('---------pub topics--------')

    pubs = {}

    for pt in pub_topics:
        print(
            'name ', pt["topic_name"],
            ' and type is ', pt["msg_type"], 
            ' at freq', pt["max_freq"]
        )
        msg_class = roslib.message.get_message_class(pt['msg_type'].split('.')[0])
        pub = rospy.Publisher(pt['topic_name'], msg_class, queue_size=10)
        def timerCallback(event, pub=pub, msg_class=msg_class):
            msg_instance = msg_class()
            pub.publish(msg_instance)
        timer = rospy.Timer(
            rospy.Duration(1.0 / pt['max_freq']),
            timerCallback
        )
        pubs[pt['topic_name']] = (pub, timer)
    
##########sub########

    print('---------sub topics--------')

    subs = {}

    for st in sub_topics:
        print(
            'name: ', st["topic_name"],
            ' and type is ', st["msg_type"] 
        )
        msg_class = roslib.message.get_message_class(st['msg_type'].split('.')[0])
        def callback(msg, topic_name=st['topic_name']):
			return 0
#            print('Received a message from', topic_name, ': ', msg)
        sub = rospy.Subscriber(st['topic_name'], msg_class, callback)
        subs[st['topic_name']] = sub

    rospy.spin()

