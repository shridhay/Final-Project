"""
Base class for rendering faces for Shutter with PySide. Other faces should inherit PySideFace's structure.
"""
import os
import numpy as np
from PySide2 import QtGui, QtCore, QtWidgets
from shutter_face.face_params import fp_v1, fp_v2
import signal
import logging

DATA_PATH = os.path.join(os.path.dirname(__file__), 'data')

class FlatFace(QtWidgets.QWidget):

    def __init__(self, show_window=True, blink=True, face_scale=1.0, data_path=DATA_PATH, res="v1", logger=logging.warning):
        """
        Constructor
        :param show_window: show the window upon initialization
        :param blink: enable blinking?
        :param face_scale: scaling for the face
        :param data_path: path to data folder with expression masks
        :param res: string identifying the screen resolution version (must be v1 or v2)
        :param logger: logging object to use for messages; recommended rospy.logwarn when running as ROS node
        """
        super(FlatFace, self).__init__()

        # optionally connect with provided logger
        self.logger = logger

        # Hack to close the interface with control-C on the command line
        # see https://stackoverflow.com/questions/5160577/ctrl-c-doesnt-work-with-pyqt
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # general app parameters
        self.full_screen = False                            # enable fullscreen mode when the app starts?
        self.bg_color = (100, 100, 100)                     # background color

        # apply screen version
        fp = fp_v1 if res == "v1" else fp_v2

        # drawing parameters
        self.face_scale = face_scale
        self.face_width = fp.width*face_scale     # face width
        self.face_height = fp.height*face_scale   # face height
        stroke_width = 12 if res == 'v1' else 17
        self.eye_stroke = stroke_width*face_scale           # eye stroke width
        self.eye_dia = fp.eye_diameter*face_scale      # eye diameter
        self.pupil_dia = fp.pupil_diameter*face_scale  # pupil diameter
        self.eye_stroke_color = (0, 0, 0)                   # eye stroke color
        self.sclera_color = (200, 200, 200)                 # sclera color
        self.pupil_color = (0, 0, 0)                        # pupil color
        self.face_center_x = self.face_width * 0.5 + fp.horizontal_offset * face_scale  # horizontal center for the eyes
        self.face_center_y = self.face_height * 0.5              # vertical center for the eyes
        self.eye_separation = fp.eye_separation*face_scale  # eyes separation
        self.left_eye_pos = np.array([self.face_center_x + self.eye_separation, self.face_center_y])  # position of the left eye
        self.right_eye_pos = np.array([self.face_center_x - self.eye_separation, self.face_center_y]) # position of the right eye
        self.left_pupil_pos = np.array([0, 0])              # left pupil position
        self.right_pupil_pos = np.array([0, 0])             # right pupil position

        # blinking parameters
        self.enable_blinking = blink                        # enable blinking
        self.is_blinking = False                            # are the eyes blinking?
        self.blinking_timer = None                          # timer for next blink
        self.average_blink_wait = 5.2                       # average wait between blinks (in seconds)
        self.std_blink_wait = 3                             # std for the time between blinks (in seconds)
        self.min_blink_wait = 1                             # minimum time between blinks (in seconds)
        self.blink_duration = 0.3                           # duration of a blink (in seconds)

        # each expression is mapped to a certain string ('neutral' is default)
        self.expression_index = "neutral"

        # setup labels for facial expressions
        self.data_path = data_path
        self.expression_labels = {expr: (QtWidgets.QLabel(self), QtWidgets.QLabel(self))
                                  for expr in fp.expressions.keys()}
        self.expr_params = fp.expressions

        self.setup()
        self.setConnections()

        # update blinking variables if blinking is enabled
        if self.enable_blinking:
            # blink once to start the timer...
            self.blinking_timeout()

        # show the window
        if show_window:
            self.setGeometry(0, 0, self.face_width, self.face_height)  # x, y, w, h
            self.setWindowTitle('PySideFace')
            self.show()

    def setup(self):
        """Setup method. Prepare the interface and anything that is necessary for proper operation of the GUI"""
        pass

    def setConnections(self):
        """Set constant connections"""
        pass

    def setBackgroundColor(self, red, green, blue):
        """
        Set background color based on RGB values
        :param red: red color (over 255)
        :param green: green color (over 255)
        :param blue: blue color (over 255)
        """
        assert 0 <= red <= 255, "The red input to SetBackgroundColor() must be in [0,255]"
        assert 0 <= green <= 255, "The green input to SetBackgroundColor() must be in [0,255]"
        assert 0 <= blue <= 255, "The blue input to SetBackgroundColor() must be in [0,255]"
        self.bg_color = (red, green, blue)

    def setFullScreen(self, value):
        """
        Set fullscreen
        :param value: boolean indicating if full screen mode is desired
        """
        assert isinstance(value, bool), "The input value to setFullScreen() must be a boolean"
        self.full_screen = value

    def setBlink(self, value):
        """
        Set blinking mode
        :param value: boolean indicating if we want the eyes to blink every once in a while
        """
        assert isinstance(value, bool), "The input value to setBlink() must be a boolean"
        self.blink = value

    def paintEvent(self, event):
        """
        Paint window. Setup environment for rendering the face.
        :param event: event that triggered painting
        """
        qp = QtGui.QPainter()
        qp.begin(self)

        # HACK because the pixels are not square in shutter's screen
        # The physical dimensions of the screen are 154.08 x 85.92 mm (see https://cdn-shop.adafruit.com/product-files/2407/c3003.pdf)
        # And the native resolution of the screen is 800 x 480 pixels.
        # Given the physical dimensions, the height of the screen should be something like 446.11 pixels for 800 pixels wide.
        # Instead of asking for a weird screen resolution, here we scale the graphics through Qt.
        qp.scale(1.0, 1.07)

        # Request for smooth graphics
        qp.setRenderHint(QtGui.QPainter.Antialiasing)

        # Now we draw the content
        self.drawExpressions()
        self.draw(event, qp)

        qp.end()

    def drawExpressions(self):
        """
        Draw the overlays by sourcing the files and attaching them to labels
        """
        # positive x is right positive y is down
        for idx in self.expression_labels.keys():
            expr = self.expr_params[idx]
            overlay_path = os.path.join(self.data_path, expr.fname)
            self.expression_labels[idx][0].setGeometry(expr.left_x*self.face_scale, expr.y*self.face_scale,
                                                       expr.w*self.face_scale, expr.h*self.face_scale)
            self.expression_labels[idx][1].setGeometry(expr.right_x*self.face_scale, expr.y*self.face_scale,
                                                       expr.w*self.face_scale, expr.h*self.face_scale)
            pixmap_left1 = QtGui.QPixmap(overlay_path)
            aspect_mode = QtCore.Qt.KeepAspectRatio if expr.keep_aspect else QtCore.Qt.IgnoreAspectRatio
            pixmap_left = pixmap_left1.scaled(expr.scale_w*self.face_scale, expr.scale_h*self.face_scale, aspectMode=aspect_mode)
            pixmap_right = pixmap_left.transformed(QtGui.QTransform().scale(-1, 1))

            self.expression_labels[idx][0].setPixmap(pixmap_left)
            self.expression_labels[idx][1].setPixmap(pixmap_right)

        # hide all overlays until requested
        self.hide_overlays()

    def hide_overlays(self):
        """
        Helper method to hide all emotion overlays
        """
        for labels in self.expression_labels.values():
            labels[0].hide()
            labels[1].hide()

    def change_relative_pupil_positions(self, lx, ly, rx, ry):
        """
        Change the relative position of the pupils on the eyes
        :param lx: horizontal position for the left pupil (in [-1, 1])
        :param ly: vertical position for the left pupil (in [-1, 1])
        :param rx: horizontal position for the right pupil (in [-1, 1])
        :param ry: vertical position for the right pupil (in [-1, 1])
        """
        def check_range(val):
            if val > 1.0: val = 1.0
            elif val < -1.0: val = -1.0
            return val

        # cap the input values just in case
        lx = check_range(lx)
        ly = check_range(ly)
        rx = check_range(rx)
        ry = check_range(ry)

        # compute new pupils positions
        max_distance = (self.eye_dia - self.pupil_dia) * 0.5
        self.left_pupil_pos[0] = lx * max_distance
        self.left_pupil_pos[1] = ly * max_distance
        self.right_pupil_pos[0] = rx * max_distance
        self.right_pupil_pos[1] = ry * max_distance

        self.update()

    def draw(self, event, qpainter):
        """
        Main draw function
        :param event: event that triggered the paint event
        :param qpainter: qpainter object to paint with
        """
        # draw the background
        p = self.palette()
        p.setColor(self.backgroundRole(), QtGui.QColor(self.bg_color[0], self.bg_color[1], self.bg_color[2]))
        self.setPalette(p)

        # render the eyes
        if self.is_blinking or self.expression_index == 'blink':
            # hide overlays when blinking
            self.hide_overlays()
            # draw blink
            self.drawBlink(qpainter, self.left_eye_pos)
            self.drawBlink(qpainter, self.right_eye_pos)
            return
        self.drawEye(qpainter, self.left_pupil_pos, self.left_eye_pos)
        self.drawEye(qpainter, self.right_pupil_pos, self.right_eye_pos)

        # show overlays according to the expression_index value
        if self.expression_index == 'neutral':
            self.hide_overlays()
            return
        try:
            self.expression_labels[self.expression_index][0].show()
            self.expression_labels[self.expression_index][1].show()
        except KeyError as exc:
            self.hide_overlays()
            self.logger(f'Could not find requested expression {exc}, fallback to neutral')

    def blinking_timeout(self):
        """
        Start blinking!
        """
        if self.is_blinking:
            # if it was blinking before, then we disable blinking and start a new timer for the next blink
            self.is_blinking = False
            delay = np.random.normal(self.average_blink_wait, self.std_blink_wait)
            if delay < self.min_blink_wait:
                delay = self.min_blink_wait

        else:
            # if it was not blinking, we now blink for a short period of time
            self.is_blinking = True
            delay = self.blink_duration

        self.blinking_timer = QtCore.QTimer.singleShot(int(delay * 1000), self.blinking_timeout)
        self.update()

    def keyPressEvent(self, e):
        """
        Handle key press events
        :param e: event
        :note By default, this function closes the window if ESC is pressed.
        """
        if e.key() == QtCore.Qt.Key_Escape:
            self.close()

    def drawEye(self, qpainter, pupil_center, eye_center):
        """
        Draw eye
        :param qpainter: QPainter object in which the eyes should be drawn over
        :param pupil_center: 2D center of the pupil in the eye coordinate frame (numpy array)
        :param eye_center: 2D center of the eye in the screen coordinate frame (numpy array)
        """
        assert isinstance(pupil_center, (np.ndarray, np.generic)), "The input pupil_center should be a numpy array"
        assert isinstance(eye_center, (np.ndarray, np.generic)), "The input eye_center should be a numpy array"

        # helper vars
        ewdiv2 = self.eye_dia * 0.5                 # half of the width of the eye
        epdiv2 = self.pupil_dia * 0.5               # half of the width of the pupil
        pupil_in_eye = pupil_center + eye_center    # position of the pupil in the eye

        # first, draw sclera of the eye
        color = QtGui.QColor(self.eye_stroke_color[0], self.eye_stroke_color[1], self.eye_stroke_color[2])
        pen = QtGui.QPen(color, self.eye_stroke)
        qpainter.setPen(pen)

        qpainter.setBrush(QtGui.QColor(self.sclera_color[0], self.sclera_color[1], self.sclera_color[2]))
        qpainter.drawEllipse(eye_center[0] - ewdiv2, eye_center[1] - ewdiv2, self.eye_dia, self.eye_dia)

        # finally, draw the pupil
        qpainter.setPen(QtCore.Qt.NoPen)

        qpainter.setBrush(QtGui.QColor(self.pupil_color[0], self.pupil_color[1], self.pupil_color[2]))
        qpainter.drawEllipse(pupil_in_eye[0] - epdiv2, pupil_in_eye[1] - epdiv2, self.pupil_dia, self.pupil_dia)

    def drawBlink(self, qpainter, eye_center):
        """
        Draw blink
        :param qpainter: QPainter object in which the eyes should be drawn over
        :param eye_center: 2D center of the eye in the screen coordinate frame (numpy array)
        """
        assert isinstance(eye_center, (np.ndarray, np.generic)), "The input eye_center should be a numpy array"

        ewdiv2 = self.eye_dia*0.5              # half of the width of the eye

        # draw a simple line
        color = QtGui.QColor(self.eye_stroke_color[0], self.eye_stroke_color[1], self.eye_stroke_color[2])
        pen = QtGui.QPen(color, self.eye_stroke)
        qpainter.setPen(pen)

        qpainter.drawLine(eye_center[0] - ewdiv2, eye_center[1], eye_center[0] + ewdiv2, eye_center[1])

    def renderToImage(self):
        """
        Render the content of the widget into an image
        :return: QImage
        """
        q_image = QtGui.QImage(self.size(), QtGui.QImage.Format.Format_RGB888)
        self.render(q_image)
        return q_image


def find_shutter_screen_by_resolution(qapp, res="v1"):
    """
    Find which screen corresponds to shutter's face based on the screens' resolutions
    :param qapp: QApplication
    :return: screen index, QScreen
    """
    fp = fp_v1 if res == "v1" else fp_v2
    for i, s in enumerate(qapp.screens()):
        geo = s.geometry() # QRect
        if geo.width() == fp.width and geo.height() == fp.height:
            return i, s
    return None, None


def move_app_to_shutter_screen(qapp, qwidget, res="v1"):
    """
    Move face widget to shutter screen. First, find the screen. Then move the widget and set full screen mode.
    :param qapp: QApplication
    :param qwidget: QWidget for shutter's face
    """
    _, qscreen = find_shutter_screen_by_resolution(qapp, res)
    if qscreen is None:
        logging.warning("Failed to find shutter's screen. Are you sure it's connected?")
        return
    geo = qscreen.geometry()
    qwidget.move(geo.left(), geo.top())
    qwidget.showFullScreen()
