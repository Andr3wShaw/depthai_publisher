#!/usr/bin/env python3

import cv2

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from geometry_msgs.msg import Point, PoseStamped
from cv_bridge import CvBridge, CvBridgeError


import numpy as np


class ArucoDetector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters()

    frame_sub_topic = '/depthai_node/image/compressed'

    def __init__(self):
        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=10)


        camera_matrix_path = '/home/uavteam5/egh450/ros_ws/src/depthai_publisher/src/depthai_publisher/camera_matrix_rgb.npy'
        dis_path = '/home/uavteam5/egh450/ros_ws/src/depthai_publisher/src/depthai_publisher/distortion_coefficients_rgb.npy'
        self.aruco_id_pub = rospy.Publisher('/aruco_marker/id', Int32, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher('/aruco_pose', Point, queue_size=10 )
        self.calibration_matrix = np.load(camera_matrix_path)
        self.distortion_coefficients = np.load(dis_path)
        self.time_finished_processing = rospy.Time(0)

        self.br = CvBridge()

        if not rospy.is_shutdown():
            self.frame_sub = rospy.Subscriber(
                self.frame_sub_topic, CompressedImage, self.img_callback)

    def img_callback(self, msg_in):
        if msg_in.header.stamp > self.time_finished_processing:   
            try:
                frame = self.br.compressed_imgmsg_to_cv2(msg_in)
            except CvBridgeError as e:
                rospy.logerr(e)

            aruco = self.pose_estimation(frame)
            self.publish_to_ros(aruco)

            self.time_finished_processing = rospy.Time.now()

        # cv2.imshow('aruco', aruco)
        # cv2.waitKey(1)

    def find_aruco(self, frame):
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))

                id_msg = Int32()
                id_msg.data = marker_ID
                self.aruco_id_pub.publish(id_msg.data)
                rospy.sleep(0.5)
                
                cv2.putText(frame, str(
                    marker_ID), (top_left[0], top_right[1] - 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

        return frame


    def pose_estimation(self, frame):
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                (top_left, top_right, bottom_right, bottom_left) = marker_corner.reshape((4, 2))

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                
                cv2.putText(frame, str(
                    marker_ID), (top_left[0], top_right[1] - 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.02, self.calibration_matrix,
                                                                       self.distortion_coefficients)

                cv2.aruco.drawDetectedMarkers(frame, corners) 

                # Draw Axis
                axis_length = 0.01
                axis_points, _ = cv2.projectPoints(
                np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]),
                rvec, tvec, self.calibration_matrix, self.distortion_coefficients)
                axis_points = axis_points.astype(int)

                cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 0, 255), 2)  # X-axis (red)
                cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[2].ravel()), (0, 255, 0), 2)  # Y-axis (green)
                cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[3].ravel()), (255, 0, 0), 2)  # Z-axis (blue)
                
                rospy.loginfo(axis_points[0])
                rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))

                id_msg = Int32()
                id_msg.data = marker_ID
                self.aruco_id_pub.publish(id_msg.data)
                # self.aruco_pose_pub.publish(axis_points[0])
                rospy.sleep(0.5)
            
        return frame

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

        self.aruco_pub.publish(msg_out)


def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()
