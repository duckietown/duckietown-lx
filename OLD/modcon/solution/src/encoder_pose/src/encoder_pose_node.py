#!/usr/bin/env python3
import os
import time
from typing import Optional

import numpy as np
import rospy
import yaml
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, EpisodeStart
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import odometry_activity
import PID_controller
import PID_controller_homework


class EncoderPoseNode(DTROS):
    """
    Computes an estimate of the Duckiebot pose using the wheel encoders.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:

    Publisher:
        ~encoder_localization (:obj:`PoseStamped`): The computed position
    Subscribers:
        ~/left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            encoder ticks
        ~/right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            encoder ticks
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(EncoderPoseNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # Init the parameters
        self.resetParameters()

        self.theta_ref = np.deg2rad(0.0)  # initial reference signal for heading control activity
        self.omega = 0.0  # initializing omega command to the robot

        # nominal R and L:
        self.log("Loading kinematics calibration...")
        self.R = 0.0318  # meters, default value of wheel radius
        self.baseline = 0.1  # meters, default value of baseline
        self.read_params_from_calibration_file()  # must have a custom robot calibration

        # Used for AIDO evaluation
        self.AIDO_eval = rospy.get_param(f"/{self.veh}/AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {self.AIDO_eval}")

        # Flags for a joyful learning experience (spins only parts of this code depending on the icons pressed on the VNC desktop)
        self.ODOMETRY_ACTIVITY = False
        self.PID_ACTIVITY = False
        self.PID_EXERCISE = False

        # Active only when submitting and evaluating (PID Exercise)
        if self.AIDO_eval:
            self.PID_EXERCISE = True
            self.v_0 = 0.2
            self.log("Starting evaluation for PID lateral controller.")

        # Defining subscribers:

        # select the current activity
        rospy.Subscriber(f"/{self.veh}/activity_name", String, self.cbActivity, queue_size=1)

        rospy.Subscriber(f"/{self.veh}/PID_parameters", String, self.cbPIDparam, queue_size=1)

        # Wheel encoder subscriber:
        left_encoder_topic = f"/{self.veh}/left_wheel_encoder_node/tick"
        rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.cbLeftEncoder, queue_size=1)

        # Wheel encoder subscriber:
        right_encoder_topic = f"/{self.veh}/right_wheel_encoder_node/tick"
        rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.cbRightEncoder, queue_size=1)

        # # AIDO challenge payload subscriber
        episode_start_topic = f"/{self.veh}/episode_start"
        rospy.Subscriber(episode_start_topic, EpisodeStart, self.cbEpisodeStart, queue_size=1)

        # Odometry publisher
        self.db_estimated_pose = rospy.Publisher(
            f"/{self.veh}/encoder_localization", Odometry, queue_size=1, dt_topic_type=TopicType.LOCALIZATION
        )

        # Command publisher
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Wait until the encoders data is received, then start the controller
        self.duckiebot_is_moving = False
        self.STOP = False

        # For encoders syncronization:
        self.RIGHT_RECEIVED = False
        self.LEFT_RECEIVED = False

        self.log("Initialized.")

    def resetParameters(self):
        # Add the node parameters to the parameters dictionary
        self.delta_phi_left = 0.0
        self.left_tick_prev = None

        self.delta_phi_right = 0.0
        self.right_tick_prev = None

        # Initializing the odometry
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0

        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0

        # Initializing the PID controller parameters
        self.prev_e = 0.0  # previous tracking error, starts at 0
        self.prev_int = 0.0  # previous tracking error integral, starts at 0
        self.time = 0.0

        self.v_0 = 0.0  # fixed robot linear velocity - starts at zero so the activities start on command
        # inside VNC
        # reference y for PID lateral control activity - zero so can be set interactively at runtime
        self.y_ref = 0.0


    def cbEpisodeStart(self, msg: EpisodeStart):
        loaded = yaml.load(msg.other_payload_yaml, Loader=yaml.FullLoader)
        if "initial_pose" in loaded:
            ip = loaded["initial_pose"]
            self.resetParameters()
            self.y_prev = float(ip["y"])
            self.theta_prev = float(np.deg2rad(ip["theta_deg"]))
            if self.AIDO_eval:
                self.PID_EXERCISE = True
                self.v_0 = 0.2
                # Feed updated initial command towards ROS agent solution in 2nd episode LF-full-loop-001
                u = [self.v_0, 0.0]
                self.publishCmd(u)
        else:
            self.logwarn("No initial pose received. If you are running this on a real robot "
                         "you can ignore this message.")
            self.y_prev = 0.0
            self.theta_prev = 0.0

    # Emergency stop / interactive pane for PID activity and exercise
    def cbPIDparam(self, msg):
        PID_parameters = msg.data

        if PID_parameters == "STOP":
            self.publishCmd([0, 0])
            self.STOP = True
            self.log("STOP")
            return

        PID_parameters = PID_parameters.split(";")
        self.log(PID_parameters)

        # ref is angle in activity
        if self.PID_ACTIVITY:
            self.theta_ref = np.deg2rad(float(PID_parameters[0]))

        # ref is lateral position in exercise
        elif self.PID_EXERCISE:
            self.y_ref = float(PID_parameters[0])

        self.v_0 = float(PID_parameters[1])

        if self.STOP:
            self.publishCmd([self.v_0, self.omega])
            self.STOP = False

    def cbActivity(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """

        self.publishCmd([0, 0])
        self.PID_ACTIVITY = False
        self.ODOMETRY_ACTIVITY = False
        self.PID_EXERCISE = False

        self.log("")
        self.log(f"Received activity {msg.data}")
        self.log("")

        self.ODOMETRY_ACTIVITY = msg.data == "odometry"
        self.PID_ACTIVITY = msg.data == "pid"
        self.PID_EXERCISE = msg.data == "pid_exercise"

    def cbLeftEncoder(self, encoder_msg):
        """
        Wheel encoder callback
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # Do nothing if the activity is not set
        if not (self.ODOMETRY_ACTIVITY or self.PID_ACTIVITY or self.PID_EXERCISE):
            return

        # initializing ticks to stored absolute value
        if self.left_tick_prev is None:
            ticks = encoder_msg.data
            self.left_tick_prev = ticks
            return

        # running the DeltaPhi() function copied from the notebooks to calculate rotations
        delta_phi_left, self.left_tick_prev = odometry_activity.DeltaPhi(encoder_msg, self.left_tick_prev)
        self.delta_phi_left += delta_phi_left

        # compute the new pose
        self.LEFT_RECEIVED = True
        self.posePublisher()

    def cbRightEncoder(self, encoder_msg):
        """
        Wheel encoder callback, the rotation of the wheel.
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # Do nothing if the activity is not set
        if not (self.ODOMETRY_ACTIVITY or self.PID_ACTIVITY or self.PID_EXERCISE):
            return

        if self.right_tick_prev is None:
            ticks = encoder_msg.data
            self.right_tick_prev = ticks
            return

        # calculate rotation of right wheel
        delta_phi_right, self.right_tick_prev = odometry_activity.DeltaPhi(encoder_msg, self.right_tick_prev)
        self.delta_phi_right += delta_phi_right

        # compute the new pose
        self.RIGHT_RECEIVED = True
        self.posePublisher()

    def posePublisher(self):
        """
        Publish the pose of the Duckiebot given by the kinematic model
            using the encoders.
        Publish:
            ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
        """
        if self.STOP or not (self.LEFT_RECEIVED and self.RIGHT_RECEIVED):
            return

        # synch incoming messages from encoders
        self.LEFT_RECEIVED = self.RIGHT_RECEIVED = False

        self.x_curr, self.y_curr, theta_curr = odometry_activity.poseEstimation(
            self.R,
            self.baseline,
            self.x_prev,
            self.y_prev,
            self.theta_prev,
            self.delta_phi_left,
            self.delta_phi_right,
        )

        self.theta_curr = self.angle_clamp(theta_curr)  # angle always between 0,2pi

        # self.loging to screen for debugging purposes
        self.log("              ODOMETRY             ")
        # self.log(f"Baseline : {self.baseline}   R: {self.R}")
        self.log(f"Theta : {np.rad2deg(self.theta_curr)} deg,  x: {self.x_curr} m,  y: {self.y_curr} m")
        self.log(
            f"Rotation left wheel : {np.rad2deg(self.delta_phi_left)} deg,   Rotation right wheel : "
            f"{np.rad2deg(self.delta_phi_right)} deg"
        )
        self.log(f"Prev Ticks left : {self.left_tick_prev}   Prev Ticks right : {self.right_tick_prev}")
        # self.log(
        #     f"Prev integral error : {self.prev_int}")

        self.duckiebot_is_moving = abs(self.delta_phi_left) > 0 or abs(self.delta_phi_right) > 0

        # Calculate new odometry only when new data from encoders arrives
        self.delta_phi_left = self.delta_phi_right = 0

        # Current estimate becomes previous estimate at next iteration
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

        # Creating message to plot pose in RVIZ
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = rospy.Time.now()

        odom.pose.pose.position.x = self.x_curr  # x position - estimate
        odom.pose.pose.position.y = self.y_curr  # y position - estimate
        odom.pose.pose.position.z = 0  # z position - no flying allowed in Duckietown

        # these are quaternions - stuff for a different course!
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = np.sin(self.theta_curr / 2)
        odom.pose.pose.orientation.w = np.cos(self.theta_curr / 2)

        self.db_estimated_pose.publish(odom)

        if self.PID_ACTIVITY or self.PID_EXERCISE:  # run the contoller only in appropriate activities
            self.Controller()

    def Controller(self):
        """
        Calculate theta and perform the control actions given by the PID
        """

        time_now = time.time()
        delta_time = time_now - self.time

        self.time = time_now

        if self.duckiebot_is_moving:

            if self.PID_ACTIVITY:
                u, self.prev_e, self.prev_int = PID_controller.PIDController(
                    self.v_0, self.theta_ref, self.theta_curr, self.prev_e, self.prev_int, delta_time
                )

            elif self.PID_EXERCISE:
                u, self.prev_e, self.prev_int = PID_controller_homework.PIDController(
                    self.v_0, self.y_ref, self.y_curr, self.prev_e, self.prev_int, delta_time
                )
        else:
            u = [self.v_0, 0.0]

        self.publishCmd(u)

    def publishCmd(self, u):
        """Publishes a car command message.

        Args:
            omega (:obj:`double`): omega for the control action.
        """

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = u[0]  # v
        car_control_msg.omega = u[1]  # omega
        # save omega in case of STOP
        self.omega = u[1]

        self.pub_car_cmd.publish(car_control_msg)

    def onShutdown(self):
        super(EncoderPoseNode, self).on_shutdown()

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the
        node with the new values.
        """
        # Check file existence
        cali_file_folder = "/data/config/calibrations/kinematics/"
        fname = cali_file_folder + self.veh + ".yaml"
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            fname = cali_file_folder + "default.yaml"
            self.readFile(fname)
            self.logwarn("Kinematics calibration %s not found! Using default instead." % fname)
        else:
            self.readFile(fname)

    def readFile(self, fname):
        with open(fname, "r") as in_file:
            try:
                yaml_dict = yaml.load(in_file, Loader=yaml.FullLoader)
                self.log(yaml_dict)
                self.R = yaml_dict["radius"]
                self.baseline = yaml_dict["baseline"]
            except yaml.YAMLError as exc:
                self.logfatal("YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                rospy.signal_shutdown("")
                return

    def angle_clamp(self, theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = EncoderPoseNode(node_name="encoder_pose_node")
    # Keep it spinning
    rospy.spin()
