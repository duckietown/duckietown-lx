#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# TODO: fill in the topic we want to subscribe to
TOPIC_TO_SUBSCRIBE = "chatter"

# TODO: fill in the name of this node
NODE_NAME = "listener"

# TODO: what type of message do we expect to receive?
MSG_TYPE = String

def callback(msg):
    # You can acces the fields of a message using dot notation, just as you access 
    # fields of a Python object
    rospy.loginfo(rospy.get_caller_id() + "\nI heard %s\n", msg.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node(NODE_NAME, anonymous=True)

    rospy.Subscriber(TOPIC_TO_SUBSCRIBE, MSG_TYPE, callback)

    # spin() simply keeps Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # TODO: which of the two functions we have written in this file do we need to run?
    listener