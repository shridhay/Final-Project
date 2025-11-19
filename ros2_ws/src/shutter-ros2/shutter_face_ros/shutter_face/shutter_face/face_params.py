"""Dataclass for Shutter face parameters"""
from dataclasses import dataclass

@dataclass
class FaceParams:
    """Class for specifying Shutter face dimensions"""
    width: int  # width for face based on the native screen resolution
    height: int  # height for face based on the native screen resolution
    horizontal_offset: int  # adjustment for eyes position because the physical screen is not centered on the face
    eye_scale: float
    pupil_scale: float
    separation_scale: float
    fx: float  # horizontal focal length for the eyes
    fy: float  # vertical focal length for the eyes
    expressions: dict  # parameters for expression masks

    @property
    def eye_diameter(self) -> float:
        """size of the scleras of the eyes"""
        return self.width * self.eye_scale

    @property
    def pupil_diameter(self) -> float:
        """size of the pupils"""
        return self.eye_diameter * self.pupil_scale

    @property
    def eye_separation(self) -> float:
        """separation between the eyes"""
        return self.eye_diameter * self.separation_scale

@dataclass
class Expression:
    """parameters for expressions"""
    left_x: int
    right_x: int
    y: int
    w: int
    h: int
    scale_w: float
    scale_h: float
    keep_aspect: bool
    fname: str

expr_v1 = {'angry': Expression(left_x=142, right_x=430, y=-99, w=490, h=690, scale_w=500, scale_h=297,
                               keep_aspect=True, fname='Angry_Overlay.svg'),
           'bored': Expression(left_x=142, right_x=430, y=-99, w=490, h=690, scale_w=500, scale_h=297,
                               keep_aspect=True, fname='Bored_Overlay.svg'),
           'determined': Expression(left_x=142, right_x=430, y=-99, w=490, h=690, scale_w=500, scale_h=297,
                                    keep_aspect=True, fname='Determined_Overlay.svg'),
           'happy': Expression(left_x=141, right_x=430, y=-87, w=490, h=690, scale_w=255, scale_h=275,
                               keep_aspect=False, fname='Happy_Overlay.svg'),
           'happy2': Expression(left_x=142, right_x=430, y=-86, w=490, h=690, scale_w=255, scale_h=274,
                                keep_aspect=False, fname='Happy_Overlay_2.svg'),
           'sad': Expression(left_x=142, right_x=430, y=-86, w=490, h=690, scale_w=252, scale_h=275,
                            keep_aspect=False, fname='Sad_Overlay.svg'),
           'surprised': Expression(left_x=119, right_x=406, y=-88, w=490, h=690, scale_w=300, scale_h=270,
                                   keep_aspect=False, fname='Surprised_Overlay.svg')}

expr_v2 = {'angry': Expression(left_x=107, right_x=537, y=-45, w=490, h=690, scale_w=500, scale_h=475,
                               keep_aspect=True, fname='Angry_Overlay.svg'),
           'bored': Expression(left_x=107, right_x=537, y=-45, w=490, h=690, scale_w=500, scale_h=475,
                               keep_aspect=True, fname='Bored_Overlay.svg'),
           'determined': Expression(left_x=107, right_x=537, y=-45, w=490, h=690, scale_w=500, scale_h=475,
                                    keep_aspect=True, fname='Determined_Overlay.svg'),
           'happy': Expression(left_x=96, right_x=522, y=-24, w=490, h=690, scale_w=430, scale_h=430,
                               keep_aspect=False, fname='Happy_Overlay.svg'),
           'happy2': Expression(left_x=99, right_x=519, y=-24, w=490, h=690, scale_w=430, scale_h=430,
                                keep_aspect=False, fname='Happy_Overlay_2.svg'),
           'sad': Expression(left_x=90, right_x=537, y=-20, w=490, h=690, scale_w=420, scale_h=440,
                             keep_aspect=False, fname='Sad_Overlay.svg'),
           'surprised': Expression(left_x=73, right_x=505, y=-27, w=490, h=690, scale_w=470, scale_h=435,
                                   keep_aspect=False, fname='Surprised_Overlay.svg')}

fp_v1 = FaceParams(width=800,
                   height=480,
                   horizontal_offset=12,
                   eye_scale=0.3,
                   pupil_scale=0.5,
                   separation_scale=0.6,
                   fx=40,
                   fy=40,
                   expressions=expr_v1)

fp_v2 = FaceParams(width=1024,
                   height=600,
                   horizontal_offset=12,
                   eye_scale=0.375,
                   pupil_scale=0.5,
                   separation_scale=0.5625,
                   fx=40,
                   fy=40,
                   expressions=expr_v2)
