import numpy as np


class UnitTestMessage:
    # Test the WheelEncoderStamped messages
    def __init__(self, callback):
        from duckietown_msgs.msg import WheelEncoderStamped
        from std_msgs.msg import Header

        # creating a dummy wheel encoder message to allow testing how to read fields

        header = Header()
        header.seq = 372
        # rospy.Time.now() is the correct stamp, anyway this works only when a node is initialized
        header.stamp.secs = 1618436796
        header.stamp.nsecs = 55785179
        header.frame_id = f"agent/left_wheel_axis"

        encoder_msg = WheelEncoderStamped(
            header=header, data=4, resolution=135, type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
        )

        callback(encoder_msg)


class UnitTestOdometry:
    # Test the odometry
    def __init__(self, R, baseline_wheel2wheel, poseEstimation):

        # initial conditions
        x_prev = y_prev = theta_prev = 0

        # to store the estimates, so we can plot them
        x_prev_ = []
        y_prev_ = []
        theta_prev_ = []

        x, y, robot_rotation = poseEstimation(
            R,
            baseline_wheel2wheel,
            x_prev,
            y_prev,
            theta_prev,
            5 * np.pi / 180,  # left wheel rotates of 5 degree
            10 * np.pi / 180, # right wheel rotates of 10 degree
            )
        # given how much the robot rotates with wheels rotation of 5 and 10 degree,
        # calculate the number of steps required to do a circle.
        # this is indipendent from R and the baseline
        steps4circle = int(2 * np.pi / robot_rotation)

        # iterate steps4circle times the pose estimation
        # function to be tested.
        for _ in range(0, steps4circle):
            # save the current values of x, y and theta
            x_prev_.append(x_prev)
            y_prev_.append(y_prev)
            theta_prev_.append(theta_prev)
            x_prev, y_prev, theta_prev = poseEstimation(
                R,
                baseline_wheel2wheel,
                x_prev,
                y_prev,
                theta_prev,
                5 * np.pi / 180,
                10 * np.pi / 180,
            )

        # plot the results
        self.plot(x_prev_, y_prev_, theta_prev_)

    def plot(self, x, y, theta):
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1)

        axes.plot(x, y, "r")
        axes.set_aspect(1)

        plt.xlabel("X position")
        plt.ylabel("Y position")

        plt.title("Am I a circle?")
        plt.show()


class UnitTestHeadingPID:
    def __init__(self, R, baseline, v_0, theta_ref, gain, trim, PIDController):
        self.R = R  # wheel radius
        # distance from wheel to wheel (notice, this is 2*L as defined in the theory)
        self.L = baseline
        self.v_0 = v_0  # constant robot linear speed
        self.PIDController = PIDController  # controller being used
        self.delta_t = 0.02  # unit test simulation time step
        self.test_horizon = 10.0
        self.t1 = np.arange(0.0, self.test_horizon, self.delta_t)  # time vector
        self.theta_prev = 0  # theta initial condition of the Duckiebot
        self.theta_ref = theta_ref  # theta ref, the goal the Duckiebot has to reach
        # motor constants (scaled to simulate hardware setup)
        self.k_r_inv = (gain + trim) / 27.0
        self.k_l_inv = (gain - trim) / 27.0

    def test(self):

        omega = 0  # initial command
        prev_e = self.theta_ref-self.theta_prev  # initializing error
        prev_int = 0  # initializing integral term

        # initializing arrays to hold variables
        err_ = [] # tracking error
        theta_hat_ = [] # estimated heading
        u_r_ = [] # input to the right wheel
        u_l_ = [] # input to the left wheel

        for _ in self.t1:
            theta_hat, u_r, u_l = self.sim(omega, self.v_0, self.delta_t)  # simulate driving

            # For plotting
            theta_hat_.append(theta_hat)
            err_.append(prev_e)
            u_r_.append(u_r)
            u_l_.append(u_l)

            # Calculating wheel commands
            v_0, omega, prev_e, prev_int = self.PIDController(
                self.v_0, self.theta_ref, theta_hat, prev_e, prev_int, self.delta_t
            )

            self.v_0 = v_0

        # plot the theta_hat and the error on theta
        self.plot_pose(theta_hat_, err_, "Duckiebot heading (Theta)", "Time (s)", "Theta (Degree)")
        # plot the control inputs
        self.plot_input(u_r_, u_l_, "Control inputs", "Time (s)", "PWM")

        # reset everything for simulation wiht noise
        self.theta_prev = 0
        omega = 0
        prev_e = 0
        prev_int = 0

        err_ = []
        theta_hat_ = []
        u_r_ = []
        u_l_ = []

        # simulate with noise
        for _ in self.t1:
            theta_hat, u_r, u_l = self.sim_noise(omega, self.v_0, self.delta_t)

            theta_hat_.append(theta_hat)
            err_.append(prev_e)
            u_r_.append(u_r)
            u_l_.append(u_l)

            v_0, omega, prev_e, prev_int = self.PIDController(
                self.v_0, self.theta_ref, theta_hat, prev_e, prev_int, self.delta_t
            )

            self.v_0 = v_0

        # plot theta with noise and the error on theta
        self.plot_pose(theta_hat_, err_, "Theta with noise", "Time (s)", "Theta (Degree)")
        # plot the input to the wheels
        self.plot_input(u_r_, u_l_, "Control inputs", "Time (s)", "PWM")

    def plot_input(self, u_r, u_l, title, x_label, y_label):
        import matplotlib.pyplot as plt

        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xticks(np.arange(0, len(u_l) + 1, 1))

        # plot the control inputs
        plt.axis([0, self.test_horizon, np.min([np.min(u_r), np.min(u_l)]), np.max([np.max(u_r), np.max(u_l)])])

        plt.plot(self.t1, (u_r), "r--", self.t1, (u_l), "b--")

        plt.legend(["Right wheel", "Left wheel"])
        plt.show()

    def plot_pose(self, theta_hat_, err_, title, x_label, y_label):
        import matplotlib.pyplot as plt

        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xticks(np.arange(0, len(theta_hat_) + 1, 1))

        theta_hat_deg = []
        for el in theta_hat_:
            theta_hat_deg.append(el * 180 / np.pi)
        err_deg = []
        for el in err_:
            err_deg.append(el * 180 / np.pi)

        # plot the error and position
        plt.axis(
            [
                0,
                self.test_horizon,
                np.min([np.min(theta_hat_deg), np.min(err_deg)]),
                np.max([np.max(theta_hat_deg), np.max(err_deg)]),
            ]
        )
        plt.plot(self.t1, (theta_hat_deg), "r--", self.t1, (err_deg), "b")

        plt.legend(["Theta", "error"])
        plt.show()

    def sim(self, omega, v, time):
        omega_l = (v - 0.5 * omega * self.L) / self.R
        omega_r = (v + 0.5 * omega * self.L) / self.R

        delta_phi_left = time * omega_l
        delta_phi_right = time * omega_r

        self.theta_prev = self.theta_prev + self.R * (delta_phi_right - delta_phi_left) / (self.L)

        u_r, u_l = self.wheel_inputs(omega_r, omega_l)

        return self.theta_prev, u_r, u_l

    def sim_noise(self, omega, v, time):
        omega_l = (v - 0.5 * omega * self.L) / self.R
        omega_r = (v + 0.5 * omega * self.L) / self.R

        delta_phi_left = time * omega_l
        delta_phi_right = time * omega_r

        # variance of the additive measurement noise
        measurement_noise_variance_deg = 0.5
        measurement_noise = np.random.normal(0, np.deg2rad(measurement_noise_variance_deg))

        self.theta_prev = (
            self.theta_prev + self.R * (delta_phi_right - delta_phi_left) / (self.L) + measurement_noise
        )

        u_r, u_l = self.wheel_inputs(omega_r, omega_l)

        return self.theta_prev, u_r, u_l

    def wheel_inputs(self, omega_r, omega_l):
        u_r = omega_r * self.k_r_inv
        u_l = omega_l * self.k_l_inv

        u_r = np.max([np.min([u_r, 1]), -1])
        u_l = np.max([np.min([u_l, 1]), -1])

        return u_r, u_l


class UnitTestPositionPID:
    def __init__(self, R, baseline, v_0, y_ref, gain, trim, PIDController):
        self.R = R
        self.L = baseline
        self.PIDController = PIDController
        self.delta_t = 0.2
        self.test_horizon = 50.0
        self.t1 = np.arange(0.0, self.test_horizon, self.delta_t)
        self.theta_prev = 0
        self.y_prev = 0
        self.y_ref = y_ref
        self.v_0 = v_0

        self.k_r_inv = (gain + trim) / 27.0  # from the kinematics node
        self.k_l_inv = (gain - trim) / 27.0  # from the kinematics node

    def test(self):
        omega = 0
        prev_e = 0
        prev_int = 0

        err_ = []
        y_hat_ = []
        u_r_ = []
        u_l_ = []

        for _ in self.t1:
            y_hat, u_r, u_l = self.sim(omega, self.v_0, self.delta_t)

            y_hat_.append(y_hat)
            err_.append(prev_e)
            u_r_.append(u_r)
            u_l_.append(u_l)

            v_0, omega, prev_e, prev_int = self.PIDController(
                self.v_0, self.y_ref, y_hat, prev_e, prev_int, self.delta_t
            )

            self.v_0 = v_0

        self.plot_pose(y_hat_, err_, "No noise", "Time (s)", "y (m)")
        self.plot_input(u_r_, u_l_, "Control inputs", "Time (s)", "PWM")

        self.theta_prev = 0
        self.y_prev = 0
        omega = 0
        prev_e = 0
        prev_int = 0

        err_ = []
        y_hat_ = []
        u_r_ = []
        u_l_ = []

        for _ in self.t1:
            y_hat, u_r, u_l = self.sim_noise(omega, self.v_0, self.delta_t)

            y_hat_.append(y_hat)
            err_.append(prev_e)
            u_r_.append(u_r)
            u_l_.append(u_l)

            v_0, omega, prev_e, prev_int = self.PIDController(
                self.v_0, self.y_ref, y_hat, prev_e, prev_int, self.delta_t
            )

            self.v_0 = v_0

        self.plot_pose(y_hat_, err_, "With noise", "Time (s)", "Y (m)")
        self.plot_input(u_r_, u_l_, "Control inputs", "Time (s)", "PWM")

    def plot_input(self, u_r, u_l, title, x_label, y_label):
        import matplotlib.pyplot as plt

        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xticks(np.arange(0, len(u_l) + 1, 1))

        # plot the control inputs
        plt.axis(
            [0, self.test_horizon, np.min([np.min(u_r), np.min(u_l)]), np.max([np.max(u_r), np.max(u_l)])]
        )

        plt.plot(self.t1, (u_r), "r--", self.t1, (u_l), "b")

        plt.legend(["Right wheel", "Left wheel"])
        plt.show()

    def plot_pose(self, y_hat_, err_, title, x_label, y_label):
        import matplotlib.pyplot as plt

        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xticks(np.arange(0, len(y_hat_) + 1, 1))

        # plot the error and position
        plt.axis(
            [
                0,
                self.test_horizon,
                np.min([np.min(y_hat_), np.min(err_)]),
                np.max([np.max(y_hat_), np.max(err_)]),
            ]
        )
        plt.plot(self.t1, (y_hat_), "r--", self.t1, (err_), "b")

        plt.legend(["Y", "error"])
        plt.show()

    def sim(self, omega, v, time):
        omega_l = (v - 0.5 * omega * self.L) / self.R
        omega_r = (v + 0.5 * omega * self.L) / self.R

        delta_phi_left = time * omega_l
        delta_phi_right = time * omega_r

        self.y_prev = self.y_prev + self.R * (delta_phi_right + delta_phi_left) * np.sin(self.theta_prev) / 2

        self.theta_prev = self.theta_prev + self.R * (delta_phi_right - delta_phi_left) / (self.L)

        u_r, u_l = self.wheel_inputs(omega_r, omega_l)

        return self.y_prev, u_r, u_l

    def sim_noise(self, omega, v, time):
        omega_l = (v - 0.5 * omega * self.L) / self.R
        omega_r = (v + 0.5 * omega * self.L) / self.R

        delta_phi_left = time * omega_l
        delta_phi_right = time * omega_r

        # variance of the additive measurement noise
        # 0.5 cm variance
        measurement_noise_variance_m = 0.005
        measurement_noise = np.random.normal(0, measurement_noise_variance_m)

        self.y_prev = (
            self.y_prev
            + self.R * (delta_phi_right + delta_phi_left) * np.sin(self.theta_prev) / 2
            + measurement_noise
        )

        self.theta_prev = self.theta_prev + self.R * (delta_phi_right - delta_phi_left) / self.L

        u_r, u_l = self.wheel_inputs(omega_r, omega_l)

        return self.y_prev, u_r, u_l

    def wheel_inputs(self, omega_r, omega_l):
        u_r = omega_r * self.k_r_inv
        u_l = omega_l * self.k_l_inv

        u_r = np.max([np.min([u_r, 1]), -1])
        u_l = np.max([np.min([u_l, 1]), -1])

        return u_r, u_l
