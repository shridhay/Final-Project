#!/usr/bin/env python3
"""
Script to detect Logitech C920 ID and write to an env-hook
"""

import logging
from pathlib import Path

def find_logitech_webcam():
    """
    Detect the ID of the FIRST attached Logitech C920 camera.
    :return: detected webcam ID
    """
    webcam_inference_logger = logging.getLogger('webcam_inference')
    # search for a matching webcam by ID
    v4l_path = Path('/dev/v4l/by-id')
    if not v4l_path.is_dir():
        webcam_inference_logger.fatal('No directory /dev/v4l/by-id exists!')
        return ""
    camera_by_id = v4l_path.glob('usb-046d_HD_Pro_Webcam_C920*-video-index0')
    try:
        shutter_webcam = next(camera_by_id).name
        webcam_inference_logger.debug('Found webcam ID: %s', shutter_webcam)
        return shutter_webcam
    except StopIteration:
        webcam_inference_logger.fatal('No matching webcam ID was found!')
        return ""

def write_webcam_id(shutter_webcam=""):
    """
    Write a detected webcam ID.
    If no webcam ID is given, write an empty string.
    :param shutter_webcam: detected webcam ID
    :return: Path object to the webcam ID script
    """
    webcam_inference_logger = logging.getLogger('webcam_inference')
    # write bash script for ament env-hook
    bash_shebang = '#!/bin/bash\n'
    export_webcam = f'export SHUTTER_WEBCAM_ID=\"{shutter_webcam}\"\n'
    webcam_id_script = Path.cwd() / 'env-hooks' / 'webcam_id.bash'
    webcam_id_script.write_text(bash_shebang + export_webcam)
    webcam_inference_logger.debug('Added webcam ID as an available environment variable')
    return webcam_id_script

def write_env_hook(webcam_id_script=None):
    """
    Write env hook to dynamically source webcam ID
    :param webcam_script: Path object to the webcam ID script
    """
    webcam_inference_logger = logging.getLogger('webcam_inference')
    bash_shebang = '#!/bin/bash\n'
    source_webcam = f'source {webcam_id_script.resolve()}\n'
    webcam_hook_script = Path.cwd() / 'env-hooks' / 'webcam_hook.bash'
    webcam_hook_script.write_text(bash_shebang + source_webcam)
    webcam_inference_logger.debug('Added webcam ID as an available environment variable')

def main():
    """
    Setup logger, detect webcam ID and write to env-hook.
    """
    # setup logger
    webcam_inference_logger = logging.getLogger('webcam_inference')
    formatter = logging.Formatter('[%(levelname)s]  %(message)s')
    ch = logging.StreamHandler()
    ch.setLevel(logging.ERROR)
    ch.setFormatter(formatter)
    webcam_inference_logger.addHandler(ch)
    webcam_inference_logger.setLevel(logging.ERROR)

    detected_camera_id = find_logitech_webcam()
    webcam_id_path = write_webcam_id(detected_camera_id)
    write_env_hook(webcam_id_path)

if __name__ == '__main__':
    main()
