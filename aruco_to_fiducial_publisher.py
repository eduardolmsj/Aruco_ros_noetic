#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import TransformStamped

class ArucoToFiducialPublisher:
    def __init__(self):
        rospy.init_node('aruco_to_fiducial_publisher')

        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher('/fiducial_transforms', FiducialTransformArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.camera_frame = rospy.get_param('~camera_frame', 'usb_cam')
        self.max_ids = rospy.get_param('~max_ids', 50)  # tenta de 0 a 49

        rospy.loginfo("Aruco to Fiducial publisher started")

        while not rospy.is_shutdown():
            self.publish_transforms()
            self.rate.sleep()

    def publish_transforms(self):
        msg = FiducialTransformArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.camera_frame
        msg.image_seq = 0  # você pode incrementar isso se quiser manter sequência

        for marker_id in range(self.max_ids):
            frame_id = f"aruco_marker_{marker_id}"
            try:
                (trans, rot) = self.listener.lookupTransform(self.camera_frame, frame_id, rospy.Time(0))
                t = FiducialTransform()
                t.fiducial_id = marker_id
                t.transform.translation.x = trans[0]
                t.transform.translation.y = trans[1]
                t.transform.translation.z = trans[2]
                t.transform.rotation.x = rot[0]
                t.transform.rotation.y = rot[1]
                t.transform.rotation.z = rot[2]
                t.transform.rotation.w = rot[3]

                msg.transforms.append(t)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.publisher.publish(msg)

if __name__ == '__main__':
    try:
        ArucoToFiducialPublisher()
    except rospy.ROSInterruptException:
        pass

