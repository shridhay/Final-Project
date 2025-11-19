#!/usr/bin/env python3
"""
ROS2 node for real-time music analysis using librosa.
Subscribes to audio input and publishes beat/rhythm information.

Terminal commands:

Install Dependencies:
    pip install librosa numpy
    sudo apt install ros-humble-audio-common
Create ROS2 Package:
    (if you don't already have a ROS2 package):
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python music_analyzer

https://github.com/ros-drivers/audio_common/tree/ros2
"""

#!/usr/bin/env python3
"""
ROS2 node for real-time music analysis using librosa.
Subscribes to audio input and publishes beat/rhythm information.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import librosa
import threading
import time
from collections import deque
from std_msgs.msg import Float32MultiArray, Float32
from audio_common_msgs.msg import AudioData


class MusicAnalyzerNode(Node):
    def __init__(self):
        super().__init__('music_analyzer')
        
        # Parameters
        self.declare_parameter('audio_topic', 'audio')
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('frame_size', 2048)
        self.declare_parameter('hop_length', 512)
        self.declare_parameter('num_channels', 1)
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.frame_size = self.get_parameter('frame_size').value
        self.hop_length = self.get_parameter('hop_length').value
        self.num_channels = self.get_parameter('num_channels').value
        audio_topic = self.get_parameter('audio_topic').value
        
        # Audio buffer for processing
        self.audio_buffer = deque(maxlen=self.sample_rate * 2)  # 2 seconds
        self.buffer_lock = threading.Lock()
        
        # QoS profile for reliable audio subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to audio input
        self.audio_sub = self.create_subscription(
            AudioData,
            audio_topic,
            self.audio_callback,
            qos
        )
        
        # Publishers
        self.beat_pub = self.create_publisher(Float32, 'music/beat', 10)
        self.tempo_pub = self.create_publisher(Float32, 'music/tempo', 10)
        self.onset_pub = self.create_publisher(Float32MultiArray, 'music/onsets', 10)
        self.energy_pub = self.create_publisher(Float32, 'music/energy', 10)
        
        # Processing thread
        self.processing_active = True
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()
        
        self.get_logger().info('Music analyzer node started')
    
    def audio_callback(self, msg):
        """Receive audio data and add to buffer"""
        try:
            # Decode raw bytes as 16-bit PCM audio
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
            
            # Convert stereo to mono if needed
            if self.num_channels > 1:
                audio_data = audio_data.reshape(-1, self.num_channels)
                audio_data = np.mean(audio_data, axis=1)
            
            with self.buffer_lock:
                self.audio_buffer.extend(audio_data)
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
    
    def processing_loop(self):
        """Main processing loop for beat and rhythm detection"""
        while self.processing_active:
            try:
                with self.buffer_lock:
                    if len(self.audio_buffer) < self.sample_rate:
                        time.sleep(0.05)
                        continue
                    # Copy buffer to avoid holding lock during processing
                    audio_data = np.array(list(self.audio_buffer), dtype=np.float32)
                
                # Normalize audio
                if np.max(np.abs(audio_data)) > 0:
                    audio_data = audio_data / np.max(np.abs(audio_data))
                
                # Compute beat tracking
                self.analyze_beat(audio_data)
                
                # Compute energy
                self.analyze_energy(audio_data)
                
            except Exception as e:
                self.get_logger().error(f'Error in processing loop: {e}')
            
            time.sleep(0.01)
    
    def analyze_beat(self, audio_data):
        """Detect beats and tempo"""
        try:
            # Compute onset strength
            onset_env = librosa.onset.onset_strength(y=audio_data, sr=self.sample_rate)
            
            # Detect beats (note: returns tempo, beat_frames in this order)
            tempo, beats = librosa.beat.beat_track(y=audio_data, sr=self.sample_rate)
            
            # Publish tempo
            tempo_msg = Float32()
            tempo_msg.data = float(tempo)
            self.tempo_pub.publish(tempo_msg)
            
            # Publish beat information
            if len(beats) > 0:
                # Calculate beat strength (normalized)
                beat_strength = float(np.mean(onset_env[beats]) if len(beats) > 0 else 0.0)
                beat_msg = Float32()
                beat_msg.data = beat_strength
                self.beat_pub.publish(beat_msg)
                
                # Publish onset times (in seconds)
                onset_times = librosa.frames_to_time(beats, sr=self.sample_rate)
                onset_msg = Float32MultiArray()
                onset_msg.data = onset_times.tolist()
                self.onset_pub.publish(onset_msg)
                
                self.get_logger().debug(f'Tempo: {tempo:.1f} BPM, Beats: {len(beats)}')
        
        except Exception as e:
            self.get_logger().error(f'Error in beat analysis: {e}')
    
    def analyze_energy(self, audio_data):
        """Analyze audio energy"""
        try:
            # Compute STFT
            S = np.abs(librosa.stft(audio_data, n_fft=self.frame_size, hop_length=self.hop_length))
            
            # Compute energy as sum of magnitude spectrogram
            energy = np.sqrt(np.sum(S ** 2, axis=0))
            
            # Normalize and get mean energy
            if np.max(energy) > 0:
                energy = energy / np.max(energy)
            
            mean_energy = float(np.mean(energy))
            
            energy_msg = Float32()
            energy_msg.data = mean_energy
            self.energy_pub.publish(energy_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in energy analysis: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.processing_active = False
        self.process_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MusicAnalyzerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import numpy as np
# import librosa
# import threading
# from collections import deque
# from std_msgs.msg import Float32MultiArray, Float32
# from audio_common_msgs.msg import AudioData


# class MusicAnalyzerNode(Node):
#     def __init__(self):
#         super().__init__('music_analyzer')
        
#         # Parameters
#         self.declare_parameter('audio_topic', '/audio')
#         self.declare_parameter('sample_rate', 44100)
#         self.declare_parameter('frame_size', 2048)
#         self.declare_parameter('hop_length', 512)
        
#         self.sample_rate = self.get_parameter('sample_rate').value
#         self.frame_size = self.get_parameter('frame_size').value
#         self.hop_length = self.get_parameter('hop_length').value
#         audio_topic = self.get_parameter('audio_topic').value
        
#         # Audio buffer for processing
#         self.audio_buffer = deque(maxlen=self.sample_rate * 2)  # 2 seconds
#         self.buffer_lock = threading.Lock()
        
#         # QoS profile for reliable audio subscription
#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )
        
#         # Subscribe to audio input
#         self.audio_sub = self.create_subscription(
#             AudioData,
#             audio_topic,
#             self.audio_callback,
#             qos
#         )
        
#         # Publishers
#         self.beat_pub = self.create_publisher(Float32, '/music/beat', 10)
#         self.tempo_pub = self.create_publisher(Float32, '/music/tempo', 10)
#         self.onset_pub = self.create_publisher(Float32MultiArray, '/music/onsets', 10)
#         self.energy_pub = self.create_publisher(Float32, '/music/energy', 10)
        
#         # Processing thread
#         self.processing_active = True
#         self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
#         self.process_thread.start()
        
#         self.get_logger().info('Music analyzer node started')
    
#     def audio_callback(self, msg):
#         """Receive audio data and add to buffer"""
#         try:
#             # Convert audio data to numpy array
#             audio_data = np.array(msg.data, dtype=np.float32)
            
#             with self.buffer_lock:
#                 for sample in audio_data:
#                     self.audio_buffer.append(sample)
#         except Exception as e:
#             self.get_logger().error(f'Error processing audio: {e}')
    
#     def processing_loop(self):
#         """Main processing loop for beat and rhythm detection"""
#         while self.processing_active:
#             try:
#                 with self.buffer_lock:
#                     if len(self.audio_buffer) < self.sample_rate:
#                         continue
                    
#                     # Convert buffer to numpy array
#                     audio_data = np.array(list(self.audio_buffer), dtype=np.float32)
                
#                 # Normalize audio
#                 if np.max(np.abs(audio_data)) > 0:
#                     audio_data = audio_data / np.max(np.abs(audio_data))
                
#                 # Compute beat tracking
#                 self.analyze_beat(audio_data)
                
#                 # Compute energy
#                 self.analyze_energy(audio_data)
                
#             except Exception as e:
#                 self.get_logger().error(f'Error in processing loop: {e}')
            
#             # Small sleep to avoid busy waiting
#             rclpy.spin_once(None, timeout_sec=0.01)
    
#     def analyze_beat(self, audio_data):
#         """Detect beats and tempo"""
#         try:
#             # Compute onset strength
#             onset_env = librosa.onset.onset_strength(y=audio_data, sr=self.sample_rate)
            
#             # Detect beats
#             beats, tempo = librosa.beat.beat_track(y=audio_data, sr=self.sample_rate)
            
#             # Publish tempo
#             tempo_msg = Float32()
#             tempo_msg.data = float(tempo)
#             self.tempo_pub.publish(tempo_msg)
            
#             # Publish beat information
#             if len(beats) > 0:
#                 # Calculate beat strength (normalized)
#                 beat_strength = float(np.mean(onset_env[beats]) if len(beats) > 0 else 0.0)
#                 beat_msg = Float32()
#                 beat_msg.data = beat_strength
#                 self.beat_pub.publish(beat_msg)
                
#                 # Publish onset times (in seconds)
#                 onset_times = librosa.frames_to_time(beats, sr=self.sample_rate)
#                 onset_msg = Float32MultiArray()
#                 onset_msg.data = onset_times.tolist()
#                 self.onset_pub.publish(onset_msg)
                
#                 self.get_logger().debug(f'Tempo: {tempo:.1f} BPM, Beats: {len(beats)}')
        
#         except Exception as e:
#             self.get_logger().error(f'Error in beat analysis: {e}')
    
#     def analyze_energy(self, audio_data):
#         """Analyze audio energy"""
#         try:
#             # Compute STFT
#             S = np.abs(librosa.stft(audio_data, n_fft=self.frame_size, hop_length=self.hop_length))
            
#             # Compute energy as sum of magnitude spectrogram
#             energy = np.sqrt(np.sum(S ** 2, axis=0))
            
#             # Normalize and get mean energy
#             if np.max(energy) > 0:
#                 energy = energy / np.max(energy)
            
#             mean_energy = float(np.mean(energy))
            
#             energy_msg = Float32()
#             energy_msg.data = mean_energy
#             self.energy_pub.publish(energy_msg)
            
#         except Exception as e:
#             self.get_logger().error(f'Error in energy analysis: {e}')
    
#     def destroy_node(self):
#         """Cleanup on shutdown"""
#         self.processing_active = False
#         self.process_thread.join(timeout=1.0)
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = MusicAnalyzerNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()