#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import ast
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse

class CameraCalibratorNode:
    def __init__(self):
        rospy.init_node('camera_calibrator', anonymous=True)

        # ROS parameters
        shape_param = rospy.get_param("~chessboard_shape", "[7, 6]")
        if isinstance(shape_param, str):
            shape_param = ast.literal_eval(shape_param)  # safely convert string to list
        self.board_shape = tuple(shape_param)
        self.square_size = rospy.get_param("~square_size", 0.025)
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.output_file = rospy.get_param("~output_file", "/tmp/camera_calibration.yaml")
        self.num_captures = rospy.get_param("~num_captures", 10)

        # ROS setup
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher("/calibration/debug_image", Image, queue_size=1)
        self.capture_srv = rospy.Service("/capture_frame", Trigger, self.capture_callback)

        self.bridge = CvBridge()
        self.objpoints = []
        self.imgpoints = []
        self.capturing = False
        self.latest_frame = None

        self.objp = self._generate_object_points()
        rospy.loginfo("Camera calibrator initialized.")

    def _generate_object_points(self):
        objp = np.zeros((self.board_shape[0]*self.board_shape[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_shape[0], 0:self.board_shape[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_shape, None)

        if ret:
            cv2.drawChessboardCorners(frame, self.board_shape, corners, ret)
            if self.capturing and len(self.imgpoints) < self.num_captures:
                # self.imgpoints.append(corners)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))
                self.imgpoints.append(corners2)
                self.objpoints.append(self.objp)
                rospy.loginfo(f"Captured frame {len(self.imgpoints)} of {self.num_captures}")
                self.capturing = False
                if len(self.imgpoints) >= self.num_captures:
                    self.calibrate()

        # Publish debug image
        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(debug_img_msg)

    def capture_callback(self, req):
        if self.latest_frame is None:
            return TriggerResponse(success=False, message="No frame received yet.")
        self.capturing = True
        return TriggerResponse(success=True, message="Frame capture requested.")

    def calibrate(self):
        img_shape = self.latest_frame.shape[:2][::-1]
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, img_shape, None, None
        )

        # Create calibration data structure matching the desired format
        calib_data = {
            "image_width": int(img_shape[0]),
            "image_height": int(img_shape[1]),
            "camera_name": "my_webcam",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": camera_matrix.flatten().tolist()
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(len(dist_coeffs.flatten())),
                "data": dist_coeffs.flatten().tolist()
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [
                    float(camera_matrix[0, 0]), 0.0, float(camera_matrix[0, 2]), 0.0,
                    0.0, float(camera_matrix[1, 1]), float(camera_matrix[1, 2]), 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]
            }
        }

        rospy.loginfo("\n=== Calibration Results ===")
        rospy.loginfo("Camera Matrix (Intrinsic Parameters):\n%s", camera_matrix)
        rospy.loginfo("Distortion Coefficients:\n%s", dist_coeffs.ravel())
        rospy.loginfo("Reprojection Error (RMS): %.4f", ret)
        rospy.loginfo("Calibration complete:")
        rospy.loginfo("%s", calib_data)

        # Custom YAML representer to format arrays inline
        def represent_list(dumper, data):
            return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

        # Save with proper YAML formatting
        with open(self.output_file, "w") as f:
            yaml.add_representer(list, represent_list)
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False)
            # Reset the representer to avoid affecting other YAML operations
            yaml.add_representer(list, yaml.representer.SafeRepresenter.represent_list)

        rospy.loginfo(f"Calibration saved to {self.output_file}")

if __name__ == "__main__":
    try:
        node = CameraCalibratorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
