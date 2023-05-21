from typing import Dict, List

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

color_map: Dict = {"red": [1.0, 0.0, 0.0],
                   "green": [0.0, 1.0, 0.0],
                   "blue": [0.0, 0.0, 1.0],
                   "white": [1.0, 1.0, 1.0]}


def construct_led_message() -> LEDPattern:
    """
    Construct an LED Pattern message that will update the LEDs on the Duckiebot.

    Returns:
        led_pattern: An LEDPattern message. This message is defined here:
        https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown_msgs/msg/LEDPattern.msg
    """
    # We start by creating an empty LEDPattern message
    led_pattern: LEDPattern = LEDPattern()

    # We will use the rgb_vals component of this message to communicate with the LED driver node
    # This component is made up of a list of 5 ColorRGBA messages, this is a standard ROS message found here:
    # http://docs.ros.org/en/melodic/api/std_msgs/html/msg/ColorRGBA.html
    rgb_vals: List[ColorRGBA] = [ColorRGBA()] * 5

    # We'll start by setting each LED individually
    # TODO: Try out different LED values and display them in the notebook
    rgb_vals = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                ColorRGBA(r=0.5, g=0.5, b=0.0, a=1.0),
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)]

    led_pattern.rgb_vals = rgb_vals
    return led_pattern


def set_leds_color(color: str, intensity: float) -> LEDPattern:
    """
    Set all the LEDs to one color and intensity in a more efficient way.
    Args:
        color: string color from the list [red, green, blue, white]
        intensity: float intensity value 0.0-1.0
    Returns:
        led_pattern: An LEDPattern message. This message is defined here:
        https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown_msgs/msg/LEDPattern.msg
    """
    # Init the message
    led_pattern: LEDPattern = LEDPattern()
    color_list: List[ColorRGBA] = [ColorRGBA()] * 5

    # Update each LED to the input color
    for rgba in color_list:
        rgba.r = color_map[color][0]
        rgba.g = color_map[color][1]
        rgba.b = color_map[color][2]
        rgba.a = intensity

    led_pattern.rgb_vals = color_list

    return led_pattern
