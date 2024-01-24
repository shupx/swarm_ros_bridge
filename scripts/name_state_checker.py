#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Empty
import threading
import time

class NameStateChecker(object):
    def __init__(self, names, me, force, loop_time):
        self.me = me
        self.names = []
        self.force = force
        for name in names:
            if name != me:
                self.names.append(name)
        print('names are', self.names)
        print('force: ', force, ' time: ', loop_time)
        self.received_msgs = {name: False for name in self.names}
        self.lock = threading.Lock()

        for name in self.names:
            rospy.Subscriber(f"/{name}/state", String, self.callback, callback_args=name)

        self.respawn_pub = rospy.Publisher("/bridge/respawn", Empty, queue_size=10)

        rospy.Timer(rospy.Duration(loop_time), self.check_msgs)

    def callback(self, msg, name):
        with self.lock:
            self.received_msgs[name] = True
            # print(f'######## get', name)

    def check_msgs(self, event):
        with self.lock:
            if not all(self.received_msgs.values()) or self.force:
                self.respawn_pub.publish(Empty())
                rospy.loginfo("Not all messages received, sending respawn signal.")
            for name in self.names:
                self.received_msgs[name] = False

if __name__ == '__main__':
    try:
        rospy.init_node('name_state_checker')

        me = rospy.get_param('~name', '')
        names = rospy.get_param('~names', '')
        force = rospy.get_param('~force_respawn', False)
        loop_time = rospy.get_param('~loop_time', 5)

        checker = NameStateChecker(names, me, force, loop_time)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
