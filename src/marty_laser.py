#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import marty_msgs.msg
import math
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError

class LaserNode(object):
    """Laser range finder ROS node for Marty

        This library allows the use of a laser pointer and a camera to estimate
        range. Simply, this is achieved by measuring the positional shift of the
        laser from a calibrated centre position.

        The accuracy of your obtained results are *hugely* dependent on a number
        of factors:
            1. the suitability of your colour thresholding parameters
            2. the accuracy of your range estimation function
            3. the design of your laser and camera mount

        For 1, this rangefinder will not be able to provide you with a range
        estimation if it cannot detect the laser to begin with. There are default
        thresholding parameters provided in laser_params.yaml
        (HSV values for a red laser) but you will very likely have to tweak these
        for your own operating environment. To aid with this, you can enable
        laser_video_out and threshold_config, which will bring up a video feed
        with configuration trackbars. You can adjust the settings until you find
        one that is suitable and then replace the values in laser_params.yaml.
        Note: the HSV colour space has been inverted (to simplify the detection
        of red).

        For 2, creating a good function is dependent on good experimental practices.
        The provided function (in calc_distance()) was obtained by plotting the
        position of the centroid of the laser dot against known distances.

        For 3, having a loose mount where the laser and/or the camera can move
        relative to each other is a bad idea. The method implemented in this
        library uses movement in the y axis, so for that reason the laser and
        camera are mounted inline vertically. For best results, ensure that
        your laser and camera are *secure* and will not move relative to each
        other.

        The current implementation is dependent on a Raspberry Pi camera
        publishing images over the /marty/camera/image topic. In the interest of
        speed, the node utilises compressed images.

        To use this node as a simple rangefinder, simply execute the provided launch
        file via roslaunch. If you wish to use the described functions directly
        within your own implementation, simply creating an instace of LaserNode
        and looping (in typical ROS fashion) will be enough to start publishing data.

        Currently in-dev, an extension of this library would be to be able to
        estimate the range of a line laser. This would enable distance
        measurements in 2 and 3 dimensional space, opening up routes for 3D
        mapping and SLAM.
        """

    def __init__(self):
        """Performs set up:
            - Instantiates the node
            - Loads ROS params from parameter server
            - Setup ROS pubs and subs
            - Private variable instantiation
        """

        super(LaserNode, self).__init__()
        rospy.init_node('laser')
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)

        """Instantiates CvBridge object, used to convert ROS images to OpenCV"""
        self.bridge = CvBridge()

        """Loads ROS params"""
        self.robot_name = rospy.get_param("/name")
        self.framerate = rospy.get_param(
            self.robot_name + "/raspicam/framerate", 90)
        self.height = rospy.get_param(
            self.robot_name + "/raspicam/height", 480)
        self.width = rospy.get_param(self.robot_name + "/raspicam/width", 640)
        self.lower_threshold = rospy.get_param("/lower_threshold", [43, 70, 6])
        self.upper_threshold = rospy.get_param(
            "/upper_threshold", [82, 255, 255])
        self.kernel_val = rospy.get_param("/kernel_val", 0)
        self.video_out_param = rospy.get_param("/laser_video_out", False)
        self.laser_type = rospy.get_param("/laser_type", "dot")
        self.threshold_config = rospy.get_param("/threshold_config", False)
        self.dot_error = rospy.get_param("/horizontal_dot_error", 3)
        self.laser_max_range = rospy.get_param("/laser_max_range", 3)
        self.laser_min_range = rospy.get_param("/laser_min_range", 0.2)
        self.dot_min_radius = rospy.get_param("/dot_min_radius", 1)
        self.scan_time = rospy.get_param("/scan_time")
        self.angle_min = rospy.get_param("/angle_min")
        self.angle_max = rospy.get_param("/angle_max")
        self.range_min = rospy.get_param("/range_min")
        self.range_max = rospy.get_param("/range_max")
        self.frame_id = rospy.get_param("/frame_id", "/odom")
        self.scan_angle = abs(self.angle_min) + abs(self.angle_max)
        self.sample_rate = rospy.get_param("/sample_rate", 2)

        """Sets up ROS pubs and subs """
        self.camera_sub = rospy.Subscriber(
            "/marty/camera/image/compressed", sensor_msgs.msg.CompressedImage, self.image_cb)

        if self.laser_type == "dot":
            self.laser_dist_pub_ = rospy.Publisher(
                '/marty/laser/distance', std_msgs.msg.Float32, queue_size=10)
        elif self.laser_type == "line":
            self.laser_scan_pub_ = rospy.Publisher(
            '/marty/laser/laserscan', sensor_msgs.msg.LaserScan, queue_size=10)
        self.mask_pub_ = rospy.Publisher(
            '/marty/laser/threshold_image', sensor_msgs.msg.CompressedImage, queue_size=10)

        """Instantiates local required variables """
        # containers for images
        self.bgr8_img = np.zeros((self.height, self.width), dtype="float32")
        self.bgr8_img_inv = np.zeros(
            (self.height, self.width), dtype="float32")
        self.hsv_img = np.zeros((self.height, self.width), dtype="float32")
        self.mask_img = np.zeros((self.height, self.width), dtype="float32")

        self.kernel = np.ones((self.kernel_val, self.kernel_val), np.uint8)

        self.lower_bound = np.array(
            [self.lower_threshold[0], self.lower_threshold[1], self.lower_threshold[2]])
        self.upper_bound = np.array(
            [self.upper_threshold[0], self.upper_threshold[1], self.upper_threshold[2]])
        self.center = 0

        # error bounds in x axis for dot position
        self.horizontal_min = (self.width / 2) - self.dot_error
        self.horizontal_max = (self.width / 2) + self.dot_error

    """Callback functions for configuration trackbars """
    def modify_lower_h_1(self, trackbar_pos):
        # Hue
        self.lower_bound[0] = cv2.getTrackbarPos('H-L1', 'Mask Image')

    def modify_lower_s_1(self, trackbar_pos):
        # Saturation
        self.lower_bound[1] = cv2.getTrackbarPos('S-L1', 'Mask Image')

    def modify_lower_v_1(self, trackbar_pos):
        # Value
        self.lower_bound[2] = cv2.getTrackbarPos('V-L1', 'Mask Image')

    def modify_upper_h_1(self, trackbar_pos):
        # Hue
        self.upper_bound[0] = cv2.getTrackbarPos('H-U1', 'Mask Image')

    def modify_upper_s_1(self, trackbar_pos):
        # Saturation
        self.upper_bound[1] = cv2.getTrackbarPos('S-U1', 'Mask Image')

    def modify_upper_v_1(self, trackbar_pos):
        # Value
        self.upper_bound[2] = cv2.getTrackbarPos('V-U1', 'Mask Image')

    def modify_kernel(self, trackbar_pos):
        # Kernel size
        value = cv2.getTrackbarPos('Kernel size', 'Mask Image')
        self.kernel = np.ones((value, value), np.uint8)

    def return_camera_info(self):
        """Returns camera image information:
            - Width
            - Height
            - Framerate """
        return (self.width, self.height, self.framerate)

    def return_node_frequency(self):
        """Returns node refresh frequency """
        return self.frequency

    def return_rate(self):
        """Returns node Rate object """
        return self.rate

    def return_video_out_param(self):
        "Returns video feed parameter. Bool value."
        return self.video_out_param

    def return_threshold_config_param(self):
        """Returns config threshold parameter. Bool value."""
        return self.threshold_config

    def image_cb(self, data):
        """Callback function for received image data.
            Takes raw image data, processes it and publishes a distance estimation.

            Converts raw ROS image to OpenCV image in bgr8 encoding, inverts this
            bgr8 image, converts the inverted image to HSV colour space and applies
            a mask to filter out the desired colour. This colour inversion was
            done in order to simplify the detection of "red" in HSV colour space,
            as red is no longer split.

            The detection method is finding the contours in the image and calculating
            the centroid co-ordinate. From this co-ordinate, the distance of the
            dot can be estimated using a range estimation function.
            """
        try:
            # Convert a compressed ROS image to OpenCV format
            self.bgr8_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            # Invert image. Essentially, we can look for "cyan" instead of "red",
            # saving us searching through two threshold bounds
            self.bgr8_img_inv = cv2.bitwise_not(self.bgr8_img)
            # Convert to hsv colour space
            self.hsv_img = cv2.cvtColor(self.bgr8_img_inv, cv2.COLOR_BGR2HSV)
            # Apply a threshold mask to filter for the desired colour
            self.mask_img = cv2.inRange(
                self.hsv_img, self.lower_bound, self.upper_bound)
            # Performs morphology operations and blurring
            self.mask_img = cv2.morphologyEx(
                self.mask_img, cv2.MORPH_CLOSE, self.kernel)
            self.mask_img = cv2.medianBlur(self.mask_img, 7)
            # Finally, searches the masked image for contours i.e our dot
            self.mask_img, contours, hierarchy = cv2.findContours(
                self.mask_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if self.laser_type == "dot":
                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    moment = cv2.moments(c)
                    try:
                        # From the moments, calculate the centroid i.e centre of our laser dot
                        self.center = (
                            int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"]))
                        # If the radius of the detected dot is greater than the min...
                        if radius > self.dot_min_radius:
                            # ...check to see if within acceptable horizontal bound
                            if (self.center[0] >= self.horizontal_min) & (self.center[0] <= self.horizontal_max):
                                cv2.circle(self.bgr8_img, self.center,
                                           5, (0, 0, 255), -1)
                                # Calculates the distance of the dot and rounds to 3 d.p.
                                dist = round(self.calc_distance(int(self.center[1]), self.laser_max_range, self.laser_min_range), 3)
                                # Data!
                                self.laser_pub_data(dist)

                                # laser_msg = self.create_laser_scan_msg(dist)
                                # self.laser_scan_pub_.publish(laser_msg)
                    except Exception as e:
                        rospy.logerr(
                            "No dot detected, check thresholding parameters?")
                        pass

            elif self.laser_type == "line":
                values = list(np.nonzero(self.mask_img))
                # print(values)
                # x = np.where(values[1] < 232)
                y = np.where(values[0] > 232)
                # print(y)
                y_valid = []
                for data_point in range(0, len(y)):
                    y_valid.append(values[0][y[data_point]])
                print(y)
                # print(values[1][data_point], values[0][data_point+1])
                # cv2.circle(self.bgr8_img, (values[1][data_point], values[0][data_point+1]), 2, (0,255,0))

        except CvBridgeError as e:
            print(e)

    def calc_gradient(self, line_endpoints):
        """Calculates the gradient of a straight line between two points

            line_endpoints must contain co-ordinates in the form:
                [x1,y1,x2,y2]"""
        y = line_endpoints[3] - line_endpoints[1]
        x = line_endpoints[2] - line_endpoints[0]
        gradient = y / float(x)
        return gradient

    def create_laser_scan_msg(self, distance=0):
        """Creates a ROS LaserScan message with the parameters specified in the
            config file

            Argument distance must be an array of range values"""

        laser_msg = sensor_msgs.msg.LaserScan()
        laser_msg.header.stamp = rospy.Time.now()
        laser_msg.header.frame_id = self.frame_id
        laser_msg.angle_min = self.angle_min*(math.pi / 180)
        laser_msg.angle_max = self.angle_max*(math.pi / 180)
        laser_msg.angle_increment = (self.scan_angle/len(distance))*(math.pi / 180)
        laser_msg.time_increment = self.scan_time/len(distance)
        laser_msg.scan_time = self.scan_time
        laser_msg.range_min = self.range_min
        laser_msg.range_max = self.range_max
        laser_msg.ranges = distance

    def line_equation(self, x, gradient, intercept):
        """Returns the result of a straight line equation

            Where:
                x - x value
                gradient - gradient
                intercept - y intercept"""
        return x * gradient + intercept

    def calc_distance(self, y_coord, max_range, min_range):
        """Returns the estimated range of the detected dot

            Where:
                y_coord - the y co-ordinate of the dot's centroid
                max_range - maximum allowable range estimation
                min_range - minimum allowable range estimation"""

        # Function relating dot position in the y-axis to range
        ''' --- '''
        '''YOU WILL MOST LIKELY NEED TO CHANGE THIS TO SUIT YOUR SETUP '''
        target_distance = 0.2389192 + \
            (4512335 - 0.2389192) / (1 + (y_coord / 127.198)**22.29046)
        ''' --- '''

        # Identifies ranges to discard
        if target_distance < min_range:
            # spooky..?
            target_distance = -0.666

        if target_distance > max_range:
            # spooky
            target_distance = 666

        return target_distance

    def update_windows(self, show_windows=False, config=False):
        """Function that shows the output of image processing and optional config trackbars

            Where:-
                show_windows - Bool value, whether to show the windows or not, defaults to False
                config - Bool value, whether to show the config trackbars or not, defaults to False"""

        if show_windows is True:
            cv2.imshow("BGR8 Image", self.bgr8_img)
            # cv2.imshow("BGR8 Image inv", self.bgr8_img_inv)
            cv2.imshow("Mask Image", self.mask_img)

        if config is True:
            cv2.createTrackbar('H-L1', 'Mask Image',
                               self.lower_bound[0], 180, self.modify_lower_h_1)
            cv2.createTrackbar('S-L1', 'Mask Image',
                               self.lower_bound[1], 255, self.modify_lower_s_1)
            cv2.createTrackbar('V-L1', 'Mask Image',
                               self.lower_bound[2], 255, self.modify_lower_v_1)
            cv2.createTrackbar('H-U1', 'Mask Image',
                               self.upper_bound[0], 180, self.modify_upper_h_1)
            cv2.createTrackbar('S-U1', 'Mask Image',
                               self.upper_bound[1], 255, self.modify_upper_s_1)
            cv2.createTrackbar('V-U1', 'Mask Image',
                               self.upper_bound[2], 255, self.modify_upper_v_1)
            cv2.createTrackbar('Kernel size', 'Mask Image', len(
                self.kernel), 50, self.modify_kernel)

        cv2.waitKey(3)

    def laser_pub_data(self, data):
        """Publishes data over the /marty/laser/distance topic"""
        self.laser_dist_pub_.publish(data)

    def pub_mask(self):
        """Publishes the thresholded image over the /marty/laser/threshold_image"""
        image = self.bridge.cv2_to_compressed_imgmsg(self.mask_img)
        self.mask_pub_.publish(image)

def main():
    laser = LaserNode()
    node_rate = laser.return_rate()
    rospy.loginfo("Initialised laser node")
    (camera_width, camera_height, camera_framerate) = laser.return_camera_info()
    rospy.loginfo("Camera info (x, y, framerate): {}, {}, {}".format(
                  camera_width, camera_height, camera_framerate))

    video_out = laser.return_video_out_param()
    config = laser.return_threshold_config_param()

    while rospy.is_shutdown() is not True:
        laser.update_windows(video_out, config)
        node_rate.sleep()

if __name__ == '__main__':
    main()
