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
    """Laser range finder ROS node for Marty"""

    def __init__(self):
        super(LaserNode, self).__init__()
        rospy.init_node('laser')
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)

        self.bridge = CvBridge()

        self.robot_name = rospy.get_param("/name")
        self.framerate = rospy.get_param(
            self.robot_name + "/raspicam/framerate", 90)
        self.height = rospy.get_param(self.robot_name + "/raspicam/height", 480)
        self.width = rospy.get_param(self.robot_name + "/raspicam/width", 640)
        self.lower_threshold = rospy.get_param("/lower_threshold", [43, 70, 6])
        self.upper_threshold = rospy.get_param("/upper_threshold", [82, 255, 255])
        self.kernel_val = rospy.get_param("/kernel_val", 0)
        self.video_out_param = rospy.get_param("/laser_video_out", False)

        self.camera_sub = rospy.Subscriber(
            "/marty/camera/image/compressed", sensor_msgs.msg.CompressedImage, self.image_cb)
        self.laser_dist_pub_ = rospy.Publisher(
            '/marty/laser_dist', std_msgs.msg.Float32, queue_size=10)
        self.laser_state_pub_ = rospy.Publisher('/marty/gpios', marty_msgs.msg.GPIOs, queue_size=10)
        self.laser_scan_pub_ = rospy.Publisher('/marty/laser_scan', sensor_msgs.msg.LaserScan, queue_size=50)

        self.bgr8_img = np.zeros((self.height, self.width), dtype = "float32")
        self.bgr8_img_inv = np.zeros((self.height, self.width), dtype = "float32")
        self.hsv_img = np.zeros((self.height, self.width), dtype = "float32")
        self.mask_img = np.zeros((self.height, self.width), dtype = "float32")

        self.kernel = np.ones((self.kernel_val,self.kernel_val),np.uint8)

        self.lower_bound = np.array([self.lower_threshold[0],self.lower_threshold[1],self.lower_threshold[2]])
        self.upper_bound = np.array([self.upper_threshold[0],self.upper_threshold[1],self.upper_threshold[2]])


    def modify_lower_h_1(self, trackbar_pos):
        self.lower_bound[0] = cv2.getTrackbarPos('H-L1','Mask Image')

    def modify_lower_s_1(self, trackbar_pos):
        self.lower_bound[1] = cv2.getTrackbarPos('S-L1','Mask Image')

    def modify_lower_v_1(self, trackbar_pos):
        self.lower_bound[2] = cv2.getTrackbarPos('V-L1','Mask Image')

    def modify_upper_h_1(self, trackbar_pos):
        self.upper_bound[0] = cv2.getTrackbarPos('H-U1','Mask Image')

    def modify_upper_s_1(self, trackbar_pos):
        self.upper_bound[1] = cv2.getTrackbarPos('S-U1','Mask Image')

    def modify_upper_v_1(self, trackbar_pos):
        self.upper_bound[2] = cv2.getTrackbarPos('V-U1','Mask Image')

    def modify_kernel(self, trackbar_pos):
        value = cv2.getTrackbarPos('Kernel size','Mask Image')
        self.kernel = np.ones((value, value), np.uint8)

    def return_camera_info(self):
        return (self.width, self.height, self.framerate)

    def return_node_frequency(self):
        return self.frequency

    def return_rate(self):
        return self.rate

    def return_video_out_param(self):
        return self.video_out_param

    def image_cb(self, data):

        try:
            self.bgr8_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            self.bgr8_img_inv = cv2.bitwise_not(self.bgr8_img)
            self.hsv_img = cv2.cvtColor(self.bgr8_img_inv, cv2.COLOR_BGR2HSV)
            self.mask_img = cv2.inRange(self.hsv_img, self.lower_bound, self.upper_bound)
            self.mask_img = cv2.morphologyEx(self.mask_img, cv2.MORPH_CLOSE, self.kernel)
            # self.mask_img = cv2.GaussianBlur(self.mask_img,(5,5),0)
            self.mask_img = cv2.medianBlur(self.mask_img,7)
            # Bitwise-AND mask and original image
            # self.bgr8_img = cv2.bitwise_and(self.bgr8_img,self.bgr8_img, mask = self.mask_img)

            self.mask_img, contours, hierarchy = cv2.findContours(self.mask_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(self.bgr8_img_inv, contours, -1, (0,255,0), 3)
            center = 0
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                moment = cv2.moments(c)
                try:
                    center = (int(moment["m10"]/moment["m00"]), int(moment["m01"]/moment["m00"]))
                    if radius > 1:
                        # cv2.drawContours(self.bgr8_img, contours, -1, (0, 255, 0), 3)
                        # cv2.circle(self.bgr8_img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        # for detected in range(0, len(center)):
                        if (center[0] >= ((self.width/2) - 3)) & (center[0] <= ((self.width/2) + 3)):
                            cv2.circle(self.bgr8_img, center, 5, (0, 0, 255), -1)
                            # print(center[1])
                            dist = round(self.calc_distance(int(center[1])), 3)
                            self.laser_pub_data(dist)
                            # laser_msg = self.create_laser_scan_msg(dist)
                            # self.laser_scan_pub_.publish(laser_msg)
                except Exception as e:
                    pass

        except CvBridgeError as e:
            print(e)

    def create_laser_scan_msg(self, distance = 0):
        laser_msg = sensor_msgs.msg.LaserScan()
        laser_msg.header = rospy.Time.now()
        laser_msg.angle_min = 3.141/180
        laser_msg.angle_max = (2*3.141)/180
        laser_msg.angle_increment = 3.141/180
        laser_msg.time_increment = 0.1
        laser_msg.scan_time = 0.2
        laser_msg.range_min = 0.15
        laser_msg.range_max = 2
        laser_msg.ranges = [distance, distance, distance]
        # print(laser_msg)
        return laser_msg

    def calc_distance(self, y_coord):
        target_distance = 0.2389192 + (4512335 - 0.2389192)/(1 + (y_coord/127.198)**22.29046)
        return target_distance

    def update_windows(self, show_windows = False, trackbars = False):
        if show_windows is True:
            cv2.imshow("BGR8 Image", self.bgr8_img)
            # cv2.imshow("BGR8 Image inv", self.bgr8_img_inv)
            cv2.imshow("Mask Image", self.mask_img)

            if trackbars is True:
                cv2.createTrackbar('H-L1','Mask Image',self.lower_bound[0],180, self.modify_lower_h_1)
                cv2.createTrackbar('S-L1','Mask Image',self.lower_bound[1],255, self.modify_lower_s_1)
                cv2.createTrackbar('V-L1','Mask Image',self.lower_bound[2],255, self.modify_lower_v_1)
                cv2.createTrackbar('H-U1','Mask Image',self.upper_bound[0],180, self.modify_upper_h_1)
                cv2.createTrackbar('S-U1','Mask Image',self.upper_bound[1],255, self.modify_upper_s_1)
                cv2.createTrackbar('V-U1','Mask Image',self.upper_bound[2],255, self.modify_upper_v_1)
                cv2.createTrackbar('Kernel size','Mask Image',len(self.kernel),50, self.modify_kernel)

            cv2.waitKey(5)

    def laser_pub_data(self, data):
        self.laser_dist_pub_.publish(data)

    def laser_pub_state(self, state):
        self.laser_state_pub_.publish(state)

    def im_pub(self, data):
        self.im_pub_.publish(data)

def main():
    laser = LaserNode()
    node_rate = laser.return_rate()
    rospy.loginfo("Initialised laser node")
    (camera_width, camera_height, camera_framerate) = laser.return_camera_info()
    rospy.loginfo("Camera info (x, y, framerate): %s, %s, %s",
                  camera_width, camera_height, camera_framerate)
    video_out = laser.return_video_out_param()
    while rospy.is_shutdown() is not True:
        laser.update_windows(video_out, True)
        node_rate.sleep()


if __name__ == '__main__':
    main()
