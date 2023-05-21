#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import LEDPattern

from solution import publish_leds


class LedPublisherNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LedPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")

        # LED publisher
        self.pub_leds = rospy.Publisher(
            "~led_pattern",
            LEDPattern,
            queue_size=1
        )
        self.log("Initialized.")

    def fade_through_colors(self):
        while not self._is_shutdown:
            for color in ["red", "green", "blue", "white"]:
                for intensity in [0.25, 0.5, 0.75, 1.0]:
                    message = publish_leds.set_leds_color(color, intensity)
                    self.pub_leds.publish(message)
                    rospy.sleep(1.)

    def on_shutdown(self):
        super(LedPublisherNode, self).on_shutdown()


if __name__ == "__main__":
    # Initialize the node
    leds_publisher_node = LedPublisherNode(node_name="led_publisher_node")

    # Fade through the defined colors
    leds_publisher_node.fade_through_colors()




