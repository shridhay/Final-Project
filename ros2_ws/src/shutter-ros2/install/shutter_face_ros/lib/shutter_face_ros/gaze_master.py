#!/usr/bin/env python3
# Node to command the eyes on the robot based on a desired 3D gaze direction
import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import tf2_ros
from shutter_face_ros.msg import PupilsLocation
from shutter_face.face_params import fp_v1 as fp

class GazeMaster(Node):
    """
    Gaze master node. Converts 3D gaze directions into 2D pupil positions.
    """
    def __init__(self):
        super().__init__('gaze_master')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # eye parameters (we assume both eyes have the same parameters)
        self.declare_parameter('fx', fp.fx)
        self.declare_parameter('fy', fp.fy)
        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.get_logger().info("fx={}, fy={}".format(self.fx, self.fy))

        # get static transforms from head to eyes (this way we only
        self.left_trans = None
        self.right_trans = None
        while self.left_trans is None and self.right_trans is None:
            if not rclpy.ok():
                return
            try:
                self.left_trans = self.tfBuffer.lookup_transform("left_eye", "head_link", Time(), timeout=Duration(seconds=1.0))
                self.right_trans = self.tfBuffer.lookup_transform("head_link", "right_eye", Time(), timeout=Duration(seconds=1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().warn("Waiting for left/right eye transforms from the head_link. Is the robot's URDF being broadcasted?")
                timeout_t = time.time() + 3.0
                while rclpy.ok() and time.time() < timeout_t:
                    rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Got eye static transforms")

        # publishers
        self.pupils_pub = self.create_publisher(PupilsLocation, "gaze/pupils_location", 5)
        self.left_pub = self.create_publisher(PointStamped, "gaze/left_coordinate_points", 5)

        # subscribers
        self.sub = self.create_subscription(PointStamped, "gaze/coordinate_points", self.callback, 10)

        # do nothing.. just wait for gaze commands (spun in main)

    def callback(self, point_msg):

        try:
            trans = self.tfBuffer.lookup_transform("head_link", point_msg.header.frame_id, Time(), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error("Failed to transform point in {} frame to head_link:\n{}".format(point_msg.header.frame_id, e))
            return

        # compute the target location from each of the eyes
        target_point = tf2_geometry_msgs.do_transform_point(point_msg, trans)
        left_point = tf2_geometry_msgs.do_transform_point(target_point, self.left_trans)
        right_point = tf2_geometry_msgs.do_transform_point(target_point, self.right_trans)

        self.left_pub.publish(left_point)

        # convert the 3D target points into 2D pixel coordinates
        left_projection = self.compute_2d_projection(left_point.point)
        right_projection = self.compute_2d_projection(right_point.point)
        # print ("target {} {} {} - left {} {} {} - projection {} {}".format(target_point.point.x,
        #                                                                    target_point.point.y,
        #                                                                    target_point.point.z,
        #                                                                    left_point.point.x,
        #                                                                    left_point.point.y,
        #                                                                    left_point.point.z,
        #                                                                    left_projection[0],
        #                                                                    left_projection[1]))

        # finally publish the result
        msg = PupilsLocation()
        msg.header = point_msg.header
        msg.left_eye.x = left_projection[0]
        msg.left_eye.y = left_projection[1]
        msg.right_eye.x = right_projection[0]
        msg.right_eye.y = right_projection[1]
        self.pupils_pub.publish(msg)


    def compute_2d_projection(self, point):
        """
        Helper function to compute 2D projection of a point
        :param point: 3D point seen from the center of the camera
        :return: 2D point
        """
        px = self.fx * point.x / -point.z # we flip z because the eyes cameras are in the 3D model poining backwards to match the Qt coordinate frame
        py = self.fy * point.y / -point.z
        return px, py


if __name__ == '__main__':
    rclpy.init(args=None)
    node = GazeMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
