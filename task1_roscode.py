#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np

calibration_data = np.load('/home/sudeesh/Technocrats/Task1/calibration_data.npz')
camera_matrix = calibration_data['mtx']
dist_coeffs = calibration_data['dist']

class balltrackernode(Node):
    def __init__(self) :
        super().__init__('balltrackernode')
        self.subscription = self.create_subscription(Image, 'camera_topic', self.image_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'ball_position' , 10)
        self.bridge = CvBridge()
        self.object_radius = 10
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        gray_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_frame, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=25, param1=50, param2=40, minRadius=2, maxRadius=30
        )
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle

                cv2.circle(undistorted_frame, (x, y), r, (0, 255, 0), 4)
                depth = (self.object_radius * camera_matrix[0, 0]) / (2 * r)
                
        ball_pose = PoseStamped()
        ball_pose.header.frame_id = 'base_link'
        ball_pose.pose.position.x = x
        ball_pose.pose.position.y = y
        ball_pose.pose.position.z = depth
        self.ball_position_pub.publish(ball_pose)

def main(args = None):
    rclpy.init(args=args)
    node = balltrackernode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()