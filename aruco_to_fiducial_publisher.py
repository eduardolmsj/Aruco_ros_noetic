#!/usr/bin/env python3

import rospy
from aruco_msgs.msg import MarkerArray
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

from geometry_msgs.msg import Pose

class ArucoToFiducialPublisher:
    def __init__(self):
        rospy.init_node('aruco_to_fiducial_publisher')

        self.camera_frame = rospy.get_param('~camera_frame', 'usb_cam')
        self.marker_topic = rospy.get_param('~marker_topic', '/aruco_marker_publisher/markers')

        self.publisher = rospy.Publisher('/fiducial_transforms', FiducialTransformArray, queue_size=10)
        rospy.Subscriber(self.marker_topic, MarkerArray, self.callback)

        rospy.loginfo(f"Subscribed to {self.marker_topic}")
        rospy.spin()

    def callback(self, marker_array_msg):
        msg = FiducialTransformArray()
        msg.header = marker_array_msg.header
        msg.image_seq = marker_array_msg.header.seq  # You can handle sequence however you like

        for marker in marker_array_msg.markers:
            fid = FiducialTransform()
            fid.fiducial_id = marker.id
            fid.transform.translation.x = marker.pose.pose.position.x
            fid.transform.translation.y = marker.pose.pose.position.y
            fid.transform.translation.z = marker.pose.pose.position.z
            fid.transform.rotation = marker.pose.pose.orientation

            msg.transforms.append(fid)

        if msg.transforms:
            self.publisher.publish(msg)

if __name__ == '__main__':
    try:
        ArucoToFiducialPublisher()
    except rospy.ROSInterruptException:
        pass
