#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import (
    SegmentList,
    LanePose,
    BoolStamped,
    Twist2DStamped,
    FSMState,
    WheelEncoderStamped,
    EpisodeStart,
)
from solution.lane_filter import LaneFilterHistogram
from sensor_msgs.msg import Image
import os
import numpy as np
from cv_bridge import CvBridge


class HistogramLaneFilterNode(DTROS):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the center of the lane.
    It gets the segments extracted by the line_detector as input and output the lane pose estimate.


    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~filter (:obj:`list`): A list of parameters for the lane pose estimation filter
        ~debug (:obj:`bool`): A parameter to enable/disable the publishing of debug topics and images

    Subscribers:
        ~segment_list (:obj:`SegmentList`): The detected line segments from the line detector
        ~(left/right)_wheel_encoder_node/tick (:obj: `WheelEncoderStamped`): Information from the wheel encoders\
        ~episode_start (:obj: `EpisodeStart`): The signal that a new episode has started 

    Publishers:
        ~lane_pose (:obj:`LanePose`): The computed lane pose estimate
        ~belief_img (:obj:`Image`): A visualization of the belief histogram as an image

    """

    def __init__(self, node_name):
        super(HistogramLaneFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        veh = os.getenv("VEHICLE_NAME")

        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        self._debug = rospy.get_param("~debug", False)
        self._predict_freq = rospy.get_param("~predict_frequency", 30.0)

        # Create the filter
        self.filter = LaneFilterHistogram(**self._filter)

        # Subscribers
        self.sub_segment_list = rospy.Subscriber(
            "~segment_list", SegmentList, self.cbProcessSegments, queue_size=1
        )

        self.sub_encoder_left = rospy.Subscriber(
            "~left_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessLeftEncoder, queue_size=1
        )

        self.sub_encoder_right = rospy.Subscriber(
            "~right_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessRightEncoder, queue_size=1
        )

        self.sub_episode_start = rospy.Subscriber(
            f"episode_start", EpisodeStart, self.cbEpisodeStart, queue_size=1
        )

        # Publishers
        self.pub_lane_pose = rospy.Publisher(
            "~lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_belief_img = rospy.Publisher(
            "~belief_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.right_encoder_ticks = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks_delta = 0
        self.left_encoder_ticks_delta = 0
        self.last_encoder_stamp = None

        # Set up a timer for prediction (if we got encoder data) since that data can come very quickly
        rospy.Timer(rospy.Duration(1 / self._predict_freq), self.cbPredict)

        self.bridge = CvBridge()

    def cbEpisodeStart(self, msg):
        rospy.loginfo("Lane Filter Resetting")
        self.filter.initialize()

    def cbProcessLeftEncoder(self, left_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = left_encoder_msg.resolution
            self.filter.initialized = True
        self.left_encoder_ticks_delta = left_encoder_msg.data - self.left_encoder_ticks
        self.last_encoder_stamp = left_encoder_msg.header.stamp

    def cbProcessRightEncoder(self, right_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = right_encoder_msg.resolution
            self.filter.initialized = True
        self.right_encoder_ticks_delta = right_encoder_msg.data - self.right_encoder_ticks
        self.last_encoder_stamp = right_encoder_msg.header.stamp

    def cbPredict(self, event):
        # first let's check if we moved at all, if not abort
        if self.right_encoder_ticks_delta == 0 and self.left_encoder_ticks_delta == 0:
            return

        self.filter.predict(self.left_encoder_ticks_delta, self.right_encoder_ticks_delta)
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

        self.publishEstimate(self.last_encoder_stamp)

    def cbProcessSegments(self, segment_list_msg):
        """Callback to process the segments

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of processed segments

        """
        # update
        self.filter.update(segment_list_msg.segments)

        # publish
        self.publishEstimate(segment_list_msg.header.stamp)

    def publishEstimate(self, stamp):

        [d_max, phi_max] = self.filter.getEstimate()

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = True
        lanePose.status = lanePose.NORMAL

        self.pub_lane_pose.publish(lanePose)
        if self._debug:
            self.debugOutput()

    def debugOutput(self):
        """Creates and publishes debug messages"""

        # Create belief image and publish it
        belief_img = self.bridge.cv2_to_imgmsg(np.array(255 * self.filter.belief).astype("uint8"), "mono8")
        self.pub_belief_img.publish(belief_img)

    def loginfo(self, s):
        rospy.loginfo("[%s] %s" % (self.node_name, s))


if __name__ == "__main__":
    lane_filter_node = HistogramLaneFilterNode(node_name="histogram_lane_filter_node")
    rospy.spin()
