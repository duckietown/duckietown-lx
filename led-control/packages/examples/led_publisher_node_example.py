#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import LEDPattern

from solution import publish_leds


class LedPublisherNode(DTROS):

    def __init__(self, node_name):
        # Initialize the ROS node
        super(LedPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")

        # Create an LED publisher to publish LEDPattern messages on the led_pattern topic
        self.pub_leds = rospy.Publisher(
            "~led_pattern",
            LEDPattern,
            queue_size=1
        )
        self.log("Initialized.")

    def fade_through_colors(self):
        """
        Fade through a series of colors on the LEDs until the node is shut down
        """
        while not self._is_shutdown:
            for color in ["red", "green", "blue", "white"]:
                for intensity in [0.25, 0.5, 0.75, 1.0]:
                    # TODO: You will define the set_leds_color function in /solution/publish_leds.py
                    # Construct the LED message
                    message = publish_leds.set_leds_color(color, intensity)

                    # Publish the LED message and let it shine for 1 second
                    self.pub_leds.publish(message)
                    rospy.sleep(1.)

    def on_shutdown(self):
        """
        Shut down cleanly
        """
        super(LedPublisherNode, self).on_shutdown()


if __name__ == "__main__":
    # Initialize the node
    leds_publisher_node = LedPublisherNode(node_name="led_publisher_node")

    # Start fading through the defined colors
    leds_publisher_node.fade_through_colors()
