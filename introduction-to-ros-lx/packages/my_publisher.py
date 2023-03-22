#!/usr/bin/env python
# license removed for brevity

# This imports the rospy library
import rospy

# We will use a message of type String when publishing to the topic
from std_msgs.msg import String

# TODO: fill in the message type
MSG_TYPE = ...

# TODO: set a rate at which you want the node to publish its messages
PUB_RATE = ... # [Hz] 

# TODO: write a message to whoever is listening to you
MESSAGE_CONTENT = f"<WHAT DO YOU WANT TO SAY?>"

# you can change the name for the topic you will publish to with this variable
TOPIC_NAME = 'chatter'

def talker():
    
    pub = rospy.Publisher(TOPIC_NAME, MSG_TYPE)
    """
    The argument `anonymous=True` will add a random number 
    if there are already nodes with the same name
    """
    rospy.init_node('talker', anonymous=True,queue_size=10)
    rate = rospy.Rate(PUB_RATE)

    while not rospy.is_shutdown():
        # We log content for debug
        rospy.loginfo(MESSAGE_CONTENT)

        # Publishing the message using the `pub` object
        pub.publish(MESSAGE_CONTENT)

        # Resting a bit while I can
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass