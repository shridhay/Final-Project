#!/usr/bin/env python
"""Face demo with mouse and keyboard input."""

import sys
import numpy as np
import logging
import argparse

from PySide2 import QtWidgets
from PySide2.QtCore import QThread, SIGNAL, QEvent, Qt, QObject

import _init_paths
import shutter_face.flat_face as flat_face


class InteractiveFace(flat_face.FlatFace):
    """
    Simple face that blinks every once in a while and can change expression.
    """

    def __init__(self, scale=1.0, blink=True, version='v1'):
        """
        Constructor
        :param blink: Do we want the face to Blink?
        :param limit_pupil_position: limit the positions of the pupils to the scleras of the eyes?
        :
        """
        # initialize the GUI
        super(InteractiveFace, self).__init__(show_window=True, blink=blink, face_scale=scale, res=version)
        self.setMouseTracking(True)

    def setMouseTracking(self, flag):
        """
        Helper function to make all child widgets get the mouse events too
        :param flag: Boolean indicating whether we want mouse tracking or not
        """
        def recursive_set(parent):
            for child in parent.findChildren(QObject):
                try:
                    child.setMouseTracking(flag)
                except:
                    pass
                recursive_set(child)

        QtWidgets.QWidget.setMouseTracking(self, flag)
        recursive_set(self)

    def change_expression(self, expression):
        """
        Change facial expression
        :param expression: string. One of: angry, bored, determined, happy, happy2, sad, surprised
        """
        self.expression_index = expression
        self.update()

    def change_gaze(self, gaze_string):
        """
        Process gaze request
        :param gaze_string: string with 2D gaze position for the left and right eye
        :note the gaze_string should be formatted as: "{} {} {} {}".format(left_x, left_y, right_x, right_y)
        """
        tok = gaze_string.split()

        # get position relative to each eye
        left = np.array([float(tok[0]) - self.left_eye_pos[0],
                         float(tok[1]) - self.left_eye_pos[1]])
        right = np.array([float(tok[2]) - self.right_eye_pos[0],
                          float(tok[3]) - self.right_eye_pos[1]])

        if np.isnan(left[0]) or np.isnan(left[1]) or \
           np.isnan(right[0]) or np.isnan(right[1]):
            logging.error("Invalid pupil positions (left = {}, right = {})".format(left, right))
            return

        # normalize gaze direction
        left = left / np.linalg.norm(left)
        right = right / np.linalg.norm(right)

        # move the pupils
        self.change_relative_pupil_positions(left[0], left[1], right[0], right[1])
        # print("Got new gaze: {} -> {} {}".format(gaze_string, self.left_pupil_pos, self.right_pupil_pos))

        self.update()

    def keyPressEvent(self, event):
        '''
        Handle key events: change expression or exit.
        :param event: event
        '''

        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_0:
                self.change_expression('neutral')
            elif event.key() == Qt.Key_1:
                self.change_expression('angry')
            elif event.key() == Qt.Key_2:
                self.change_expression('bored')
            elif event.key() == Qt.Key_3:
                self.change_expression('determined')
            elif event.key() == Qt.Key_4:
                self.change_expression('happy')
            elif event.key() == Qt.Key_5:
                self.change_expression('happy2')
            elif event.key() == Qt.Key_6:
                self.change_expression('sad')
            elif event.key() == Qt.Key_7:
                self.change_expression('surprised')

        super(InteractiveFace, self).keyPressEvent(event)

    def mouseMoveEvent(self, event):
        """
        Handle mouse events: change gaze direction.
        :param event: event
        """
        pos = event.pos()
        x = pos.x() #- np.floor(self.face_width * 0.5)
        y = pos.y() #- np.floor(self.face_height * 0.5)
        self.change_gaze('{} {} {} {}'.format(x, y, x, y))


def main():
    """
    Main function. Run the Qt application to render the face.
    """

    parser = argparse.ArgumentParser(description='Shutter face demo.')
    parser.add_argument('--scale', type=float, default=1.0, help='scale parameter for the face')
    parser.add_argument('--version', type=str, default='v1', help='version parameter for the face')
    parser.add_argument('--disable-blink', action='store_false', help='flag to disable blinking')
    args = parser.parse_args()

    # create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # create face widget
    face_gui = InteractiveFace(scale=args.scale, blink=args.disable_blink, version=args.version)

    # move widget to shutter screen
    flat_face.move_app_to_shutter_screen(app, face_gui, res=args.version)

    # run the app
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
