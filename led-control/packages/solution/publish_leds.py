from typing import Dict, List

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

color_map: Dict = {"red": [1.0, 0.0, 0.0],
                   "green": [],
                   "blue": [],
                   "white": []}


def construct_led_message() -> LEDPattern:
    """
    Construct an LED Pattern message that will update the LEDs on the Duckiebot.

    Returns:
        led_pattern: An LEDPattern message. This message is defined here:
        https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown_msgs/msg/LEDPattern.msg
    """
    # We start by creating an empty LEDPattern message
    message: LEDPattern = LEDPattern()

    # We will use the rgb_vals component of this message to communicate with the LED driver node
    # This component is made up of a list of 5 ColorRGBA messages, this is a standard ROS message found here:
    # http://docs.ros.org/en/melodic/api/std_msgs/html/msg/ColorRGBA.html
    rgb_vals: List[ColorRGBA] = [ColorRGBA()] * 5

    # We'll start by setting each LED individually
    # TODO: Try out different LED values and display them on your Duckiebot using the notebook instructions
    rgb_vals = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                ColorRGBA(r=0.5, g=0.5, b=0.0, a=1.0),
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)]

    message.rgb_vals = rgb_vals
    return message


def set_leds_color(color: str, intensity: float) -> LedPattern:
    """
    Set all the LEDs to one color and intensity in a more efficient way.
    Args:
        color: string color from the list [red, green, blue, white]
        intensity: float intensity value 0.0-1.0
    Returns:
        led_pattern: An LEDPattern message. This message is defined here:
        https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown_msgs/msg/LEDPattern.msg
    """

    # TODO: Complete the last three components of the color map at the top of this file,
    #       then use it to create an LEDMessage that matches the input color and intensity

    return None
