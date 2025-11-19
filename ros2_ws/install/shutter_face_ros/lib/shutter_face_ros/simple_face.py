#!/usr/bin/python3
"""
Simple Shutter face with blinking option
"""
import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
from shutter_face_ros.msg import PupilsLocation
from PySide2 import QtWidgets
from PySide2.QtCore import QThread, SIGNAL
from shutter_face.flat_face import FlatFace, move_app_to_shutter_screen
from shutter_face.face_params import fp_v1, fp_v2

class SimpleFaceNode(QThread):
    """Simple Face Node. Handles all ROS communication for the face GUI"""

    def __init__(self):

        # init thread
        QThread.__init__(self)

        # init ROS client library and node
        rclpy.init(args=None)
        self.node: Node = rclpy.create_node("SimpleFaceNode")

        # declare params
        self.node.declare_parameter("limit_pupils", True)
        self.node.declare_parameter("blink", True)
        self.node.declare_parameter("move_to_shutter_screen", True)
        self.node.declare_parameter("screen_version", "v1")
        self.node.declare_parameter("blink_playback", False)

        # get params
        self.limit_pupil_position = self.node.get_parameter("limit_pupils").get_parameter_value().bool_value
        self.blink = self.node.get_parameter("blink").get_parameter_value().bool_value
        self.move_to_shutter_screen = self.node.get_parameter("move_to_shutter_screen").get_parameter_value().bool_value
        self.screen_version = self.node.get_parameter("screen_version").get_parameter_value().string_value
        self.blink_playback = self.node.get_parameter("blink_playback").get_parameter_value().bool_value

        # prioritise generated blinking over playback blinking
        if self.blink and self.blink_playback:
            self.node.get_logger().error("blink and blink_playback were both set to true, disabling blink_playback")
            self.blink_playback = False

        # Subscribers
        self.pupils_sub = self.node.create_subscription(PupilsLocation, "gaze/pupils_location", self.gaze_callback, 10)
        self.expr_sub = self.node.create_subscription(String, "gaze/expression_index", self.expression_callback, 10)
        if self.blink_playback:
            self.is_blinking = False
            self.blink_sub = self.node.create_subscription(Bool, "gaze/blink", self.blink_callback, 10)
        else:
            self.blink_pub = self.node.create_publisher(Bool, "gaze/blink", 1)

        # executor for callbacks
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def run(self):
        try:
            self.executor.spin()
        finally:
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()

    def expression_callback(self, data):
        """
        Trigger change in expression overlay based on expression_index topic
        """

        index = data.data
        self.emit(SIGNAL('change_expression(QString)'), index)

    def gaze_callback(self, gaze_msg):
        """
        Trigger change in GUI based on requested gaze direction
        :param gaze_msg: 2DGaze message
        """
        gaze_string = "{} {} {} {}".format(gaze_msg.left_eye.x, gaze_msg.left_eye.y,
                                           gaze_msg.right_eye.x, gaze_msg.right_eye.y)
        self.emit(SIGNAL('change_gaze(QString)'), gaze_string)

    def blink_callback(self, blink_msg):
        if self.is_blinking == blink_msg.data:
            return
        self.is_blinking = blink_msg.data
        self.emit(SIGNAL('change_blink(bool)'), self.is_blinking)


class SimpleFace(FlatFace):
    """
    Simple face that blinks every once in a while.
    """

    def __init__(self, scale=1.0):
        """
        Constructor
        :param scale: face scale
        """
        # initialize all ROS stuff
        self.node = SimpleFaceNode()
        self.node.start()
        # initialize the GUI
        pkg_path = get_package_share_directory('shutter_face_ros')
        super(SimpleFace, self).__init__(show_window=True, blink=self.node.blink, face_scale=scale,
                                         data_path=os.path.join(pkg_path, "shutter_face", "shutter_face", "data"),
                                         res=self.node.screen_version,
                                         logger=self.node.node.get_logger().warn if hasattr(self.node, 'node') else self.node.node.get_logger().warn)

    def change_expression(self, index):
        """
        Process expression requests
        """
        self.expression_index = index
        self.update()

    def change_gaze(self, gaze_string):
        """
        Process gaze request
        :param gaze_string: string with 2D gaze position for the left and right eye
        """
        tok = gaze_string.split()

        left_x = float(tok[0])
        left_y = float(tok[1])
        right_x = float(tok[2])
        right_y = float(tok[3])

        if np.isnan(left_x) or np.isnan(left_y) or \
           np.isnan(right_x) or np.isnan(right_y):
            self.node.node.get_logger().error("Invalid pupil positions (left = ({}, {}), right = ({}, {}))".format(left_x, left_y, right_x, right_y))
            return

        # Get the desired pupil position.
        # Note that we flip Y here because the drawing origin is in the top-left corner of the screen
        self.left_pupil_pos = np.array([left_x, left_y])  # left pupil position
        self.right_pupil_pos = np.array([right_x, right_y])  # right pupil position

        # limit the actual pupil position
        if self.node.limit_pupil_position:
            self.left_pupil_pos = self.limit_pupil(self.left_pupil_pos)
            self.right_pupil_pos = self.limit_pupil(self.right_pupil_pos)

        # print("Got new gaze: {} -> {} {}".format(gaze_string, self.left_pupil_pos, self.right_pupil_pos))

        self.update()

    def limit_pupil(self, pupil_position):
        """
        Limit the position of the pupil within an eye
        :param pupil_position: 2D pupil position within the eye
        :return: new pupil position
        """
        fp = fp_v1 if self.node.screen_version == "v1" else fp_v2
        max_distance = (fp.eye_diameter - fp.pupil_diameter)*0.5
        distance = np.linalg.norm(pupil_position)
        if distance > max_distance:
            pupil_position = pupil_position * max_distance / distance
        return pupil_position

    def change_blink(self, is_blinking):
        self.is_blinking = is_blinking
        self.update()

    def setup(self):
        super(SimpleFace, self).setup()

        # setup GUI based on ROS params
        self.enable_blinking = self.node.blink

    def setConnections(self):
        """Set connections"""
        super(SimpleFace, self).setConnections()

        # connect ROS signals with Qt
        self.connect(self.node, SIGNAL("change_gaze(QString)"), self.change_gaze)
        self.connect(self.node, SIGNAL("change_expression(QString)"), self.change_expression)
        if self.node.blink_playback:
            self.connect(self.node, SIGNAL("change_blink(bool)"), self.change_blink)

    def draw(self, event, qpainter):
        super(SimpleFace, self).draw(event, qpainter)
        if self.node.blink:
            if hasattr(self, 'is_blinking'):
                self.node.blink_pub.publish(Bool(data=self.is_blinking))


def main():
    """
    Main function. Run the Qt application to render the face.
    """
    # create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # create face widget
    face_gui = SimpleFace()

    # move widget to shutter screen
    if face_gui.node.move_to_shutter_screen:
        move_app_to_shutter_screen(app, face_gui, face_gui.node.screen_version)

    # run the app
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

