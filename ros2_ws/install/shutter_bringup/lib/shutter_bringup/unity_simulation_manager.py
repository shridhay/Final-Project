#!/usr/bin/env python3

import os
from pathlib import Path
import shutil
import subprocess

import gdown
import rclpy
from rclpy.node import Node


class UnitySimulationManager(Node):
    """
    Manager node to setup, start and stop a Shutter simulation built with Unity
    """
    def __init__(self):
        """
        Start UnitySim after checking that UnitySim exists and contains correct version.
        """
        super().__init__('simulation_manager')
        
        self.shutter_ros_path = Path(__file__).resolve().parents[2]
        self.unity_path = self.shutter_ros_path / 'UnitySim'
        self.version_fpath = self.unity_path / 'version.txt'
        self.simulation_fpath = self.unity_path / 'Shutter_MoveIt.x86_64'
        self.gdrive_id = '1j_zoamfFRhDvEe9I3aFVsxzdorCMuZzn'

        if not self.check_install():
            self.get_logger().fatal("Check for Unity simulation installation failed.")
            raise RuntimeError("Unity simulation installation check failed")
        
        executable_args = [self.simulation_fpath]
        headless = self.get_parameter_or('headless', False)
        if headless:
            executable_args.extend(['-batchmode',
                                    '-nographics'])
        self.unity_simulation_process = subprocess.Popen(executable_args)
        
        # Create a timer to keep the node alive
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Timer callback to keep the node alive"""
        pass

    def check_install(self):
        """
        Check that UnitySim is extracted to expected directory and has correct version.
        Attempts to reinstall from scratch if version is old.
        """
        if not self.unity_path.is_dir():
            self.get_logger().warn(f'{self.unity_path} is NOT an existing directory; assuming install is missing.')
            if self.install_unity_sim():
                return True
        if self.check_version():
            return True
        self.get_logger().warn('Attempting to remove and re-install UnitySim...')
        try:
            shutil.rmtree(self.unity_path)
            self.get_logger().debug(f'Removal of {self.unity_path} succeeded; attemping to install UnitySim.')
            if self.install_unity_sim():
                return True
        except Exception as e:
            self.get_logger().error(f'{e}')
        return False

    def install_unity_sim(self):
        """
        :return: True if no exceptions were raised, False otherwise
        Install the UnitySim package from Google Drive.
        """
        try:
            tarfile_fpath = self.shutter_ros_path / 'UnitySim.tgz'
            simulation_url = f'https://drive.google.com/uc?id={self.gdrive_id}'
            gdown.download(simulation_url, str(tarfile_fpath), quiet=False)
            self.get_logger().debug('Finished downloading simulation package, unzipping...')
            gdown.extractall(str(tarfile_fpath))
            self.version_fpath.write_text(self.gdrive_id)
            os.remove(tarfile_fpath)
            self.get_logger().info(f'Download and extraction to {self.unity_path} succeeded.')
            return True
        # gdown does not define any exceptions but raises Runtime and Value Errors
        except (RuntimeError, ValueError) as e:
            self.get_logger().error(f'{e}')
            return False

    def check_version(self):
        """
        :return: True if the version is correct, False otherwise
        Check if the version matches the Google Drive ID of the latest commit.
        """
        with self.version_fpath.open() as f:
            version_line = f.readline().rstrip('\n')
            if version_line == self.gdrive_id:
                return True
            self.get_logger().warn(f'Found Google Drive ID: {version_line} did not match expected ID {self.gdrive_id}.')
            return False

    def destroy_node(self):
        """Clean shutdown of the node"""
        if hasattr(self, 'unity_simulation_process'):
            self.unity_simulation_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnitySimulationManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
