import numpy as np
import cv2
from matplotlib import pyplot as plt


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


class UnitTestLMO:
    # Test the detection and estimation of lane marking orientations
    def __init__(self, LMOrientation):
        imgbgr = cv2.imread("../images/visual_control/test.png")

        theta_left, theta_right, mask_lt, mask_rt = LMOrientation(imgbgr)
        print()
        print("Function returned the following orientations for the given image:")
        print("    Left Edge:   %.2f radians (%.2f degrees)" % (theta_left, theta_left * 180 / np.pi))
        print("    Right Edge:  %.2f radians (%.2f degrees)" % (theta_right, theta_right * 180 / np.pi))

        fig = plt.figure(figsize=(20, 5))
        ax1 = fig.add_subplot(1, 3, 1)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax1.imshow(cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB))
        ax1.set_title("Do these orientations look right?"), ax1.set_xticks([]), ax1.set_yticks([])

        ax2 = fig.add_subplot(1, 3, 2)
        ax2.imshow(cv2.cvtColor(mask_lt, cv2.COLOR_BGR2RGB))
        ax2.set_title("Mask (Left)"), ax2.set_xticks([]), ax2.set_yticks([])

        ax3 = fig.add_subplot(1, 3, 3)
        ax3.imshow(cv2.cvtColor(mask_rt, cv2.COLOR_BGR2RGB))
        ax3.set_title("Mask (Right)"), ax3.set_xticks([]), ax3.set_yticks([])


class UnitTestDLM:
    # Test the detection and estimation of lane marking orientations
    def __init__(self, detect_lane_markings):
        imgbgr = cv2.imread("../../assets/images/visual_control/pic10.png")

        img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)

        left_masked_img, right_masked_img = detect_lane_markings(imgbgr)

        fig = plt.figure(figsize=(20, 5))
        ax1 = fig.add_subplot(1, 3, 1)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax1.imshow(cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB))
        ax1.set_title("Input image"), ax1.set_xticks([]), ax1.set_yticks([])

        ax2 = fig.add_subplot(1, 3, 2)
        ax2.imshow(cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB))
        ax2.imshow(left_masked_img * img, cmap="gray")
        ax2.set_title("Mask (Left)"), ax2.set_xticks([]), ax2.set_yticks([])

        ax3 = fig.add_subplot(1, 3, 3)
        ax3.imshow(right_masked_img * img, cmap="gray")
        ax3.set_title("Mask (Right)"), ax3.set_xticks([]), ax3.set_yticks([])

class UnitTestELRH:
    # Test the estimate of the robot's lane-relative heading
    def __init__(self, estimate_lane_relative_heading):
        imgbgr_straight = cv2.imread("../images/visual_control/pic10.png")
        imgbgr_turn = cv2.imread("../images/visual_control/turn.png")

        # The image-to-ground homography associated with this image (Jacopo's)
        H = np.array(
            [
                -4.137917960301845e-05,
                -0.00011445854191468058,
                -0.1595567007347241,
                0.0008382870319844166,
                -4.141689222457687e-05,
                -0.2518201638170328,
                -0.00023561657746150284,
                -0.005370140574116084,
                0.9999999999999999,
            ]
        )
        H = np.reshape(H, (3, 3))

        theta_hat, lines_left, lines_right = estimate_lane_relative_heading(imgbgr_straight)

        print("First Image: theta_hat: %.2f degrees" % (theta_hat * 180 / np.pi))
        fig = plt.figure(figsize=(20, 20))
        ax1 = fig.add_subplot(2, 2, 1)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax1.imshow(cv2.cvtColor(imgbgr_straight, cv2.COLOR_BGR2RGB))
        ax1.set_title("Input image"), ax1.set_xticks([]), ax1.set_yticks([])

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_title("Lines Projected on Ground Plane"), ax2.set_xticks([]), ax2.set_yticks([])
        if lines_left is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_left:
                [[x1, y1, x2, y2]] = line
                xy1 = np.array([[x1, y1]]).transpose()
                xy2 = np.array([[x2, y2]]).transpose()

                # Project to the ground frame
                XY1 = project_image_to_ground(H, xy1)
                XY2 = project_image_to_ground(H, xy2)

                X = np.array([XY1[0], XY2[0]])
                Y = np.array([XY1[1], XY2[1]])
                # The ground reference frame has positive X up and positivy Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax2.plot(-Y, X, "g-")

        if lines_right is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_right:
                [[x1, y1, x2, y2]] = line
                xy1 = np.array([[x1, y1]]).transpose()
                xy2 = np.array([[x2, y2]]).transpose()

                # Project to the ground frame
                XY1 = project_image_to_ground(H, xy1)
                XY2 = project_image_to_ground(H, xy2)

                X = np.array([XY1[0], XY2[0]])
                Y = np.array([XY1[1], XY2[1]])
                # The ground reference frame has positive X up and positivy Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax2.plot(-Y, X, "b-")

        theta_hat, lines_left, lines_right = estimate_lane_relative_heading(imgbgr_turn)

        print("Second Image: theta_hat: %.2f degrees" % (theta_hat * 180 / np.pi))
        ax3 = fig.add_subplot(2, 2, 3)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax3.imshow(cv2.cvtColor(imgbgr_turn, cv2.COLOR_BGR2RGB))
        ax3.set_title("Input image"), ax3.set_xticks([]), ax3.set_yticks([])

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_title("Lines Projected on Ground Plane"), ax4.set_xticks([]), ax4.set_yticks([])
        if lines_left is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_left:
                [[x1, y1, x2, y2]] = line
                xy1 = np.array([[x1, y1]]).transpose()
                xy2 = np.array([[x2, y2]]).transpose()

                # Project to the ground frame
                XY1 = project_image_to_ground(H, xy1)
                XY2 = project_image_to_ground(H, xy2)

                X = np.array([XY1[0], XY2[0]])
                Y = np.array([XY1[1], XY2[1]])
                # The ground reference frame has positive X up and positivy Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax4.plot(-Y, X, "g-")

        if lines_right is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_right:
                [[x1, y1, x2, y2]] = line
                xy1 = np.array([[x1, y1]]).transpose()
                xy2 = np.array([[x2, y2]]).transpose()

                # Project to the ground frame
                XY1 = project_image_to_ground(H, xy1)
                XY2 = project_image_to_ground(H, xy2)

                X = np.array([XY1[0], XY2[0]])
                Y = np.array([XY1[1], XY2[1]])
                # The ground reference frame has positive X up and positivy Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax4.plot(-Y, X, "b-")


def project_image_to_ground(H, x):
    """
    Args:
        H: The 3x3 image-to-ground plane homography (numpy.ndarray)
        x: An array of non-homogeneous image coordinates, one per column (numpy.ndarray)
    Returns:
        X: An array of non-homogeneous coordinates in the world (ground) frame, one per column (numpy.ndarray)
    """

    if x.shape[0] == 2:
        if x.ndim == 1:
            x = np.append(x, 1)
        else:
            x = np.vstack((x, np.ones((1, x.shape[1]))))

    X = H.dot(x)
    X = X / X[2, None]

    return X[
        0:2,
    ]

class UnitTestLM_MASKS:
    # Test the detection and estimation of lane marking orientations
    def __init__(self, get_steer_matrix_left_lane_markings, get_steer_matrix_right_lane_markings):
        imgbgr = cv2.imread("../../assets/images/visual_control/pic10.png")

        img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
        shape = tuple(imgbgr.shape[0:2])
        # Mask out above the horizon
        H = np.array([-4.137917960301845e-05, -0.00011445854191468058, -0.1595567007347241, 
              0.0008382870319844166, -4.141689222457687e-05, -0.2518201638170328, 
              -0.00023561657746150284, -0.005370140574116084, 0.9999999999999999])

        H = np.reshape(H,(3, 3))
        Hinv = np.linalg.inv(H)
        mask_ground = np.ones(img.shape, dtype=np.uint8) # TODO: CHANGE ME
        X_lim = np.array([[10,0,1]]).transpose()
        Hx = Hinv@X_lim
        u_lim = int(Hx[0]/Hx[2])
        v_lim = int(Hx[1]/Hx[2])
        mask_ground[:v_lim,:] = np.zeros_like(mask_ground[:v_lim,:])

        STEER_LEFT_LM = get_steer_matrix_left_lane_markings(shape)
        STEER_RIGHT_LM = get_steer_matrix_right_lane_markings(shape)

        STEER_LEFT_LM[STEER_LEFT_LM != 0] =1
        STEER_RIGHT_LM[STEER_RIGHT_LM != 0] =1
        
        fig = plt.figure(figsize=(20, 5))
        ax1 = fig.add_subplot(1, 3, 1)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax1.imshow(cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB))
        ax1.set_title("Input image"), ax1.set_xticks([]), ax1.set_yticks([])

        ax2 = fig.add_subplot(1, 3, 2)
        ax2.imshow(STEER_LEFT_LM * img *mask_ground, cmap="gray")
        ax2.set_title("Steering Mask (Left)"), ax2.set_xticks([]), ax2.set_yticks([])

        ax3 = fig.add_subplot(1, 3, 3)
        ax3.imshow(STEER_RIGHT_LM * img* mask_ground, cmap="gray")
        ax3.set_title("Mask (Right)"), ax3.set_xticks([]), ax3.set_yticks([])

class UnitTestSTEERING:
    # Test the detection and estimation of lane marking orientations
    def __init__(self, get_steer_matrix_left_lane_markings, get_steer_matrix_right_lane_markings, detect_lane_markings):
        imgbgr = cv2.imread("../../assets/images/visual_control/pic10.png")

        img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
        shape = tuple(imgbgr.shape[0:2])

        STEER_LEFT_LM = get_steer_matrix_left_lane_markings(shape)
        STEER_RIGHT_LM = get_steer_matrix_right_lane_markings(shape)
        
        left_lane_markings_img, right_lane_markings_img = detect_lane_markings(imgbgr)

        steering = np.sum(STEER_LEFT_LM*left_lane_markings_img)+np.sum(STEER_RIGHT_LM*left_lane_markings_img)

        normalizer = 300000
        print("Commanding steering at a rate of: ",steering/normalizer*180/np.pi," [deg/s]")