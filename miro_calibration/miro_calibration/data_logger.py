#!/usr/bin/env python3
"""
Data Logger Node for MIRO v2 Calibration System

Manages rosbag2 recording of calibration data with session management,
storage cleanup, and metadata generation.

Services:
    /calibration/start_recording (miro_msgs/StartRecording): Start recording
    /calibration/stop_recording (miro_msgs/StopRecording): Stop recording
    /calibration/get_recording_status (miro_msgs/GetRecordingStatus): Get status

Publishes:
    /calibration/recording_status (miro_msgs/RecordingStatus): Recording status at 1Hz

Parameters:
    calibration_data_dir (str): Base directory for calibration data
    compression (str): Compression algorithm (zstd, lz4, none)
    storage_preset (str): Storage backend (sqlite3, mcap)
    record_topics (list[str]): Topics to record
    max_sessions_to_keep (int): Maximum number of sessions to retain
    max_storage_gb (float): Maximum total storage size in GB
    min_disk_space_gb (float): Minimum free disk space threshold in GB
    robot_id (str): Robot identifier
    operator_name (str): Operator name
    node_version (str): Data logger version
    status_publish_rate_hz (float): Status publish rate

Requirements: REQ-5 (Comprehensive Calibration Data Logging)
"""

import json
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rosbag2_py import (
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata
)

from miro_msgs.msg import RecordingStatus
from miro_msgs.srv import StartRecording, StopRecording, GetRecordingStatus


class DataLogger(Node):
    """
    Data logger node for recording calibration sessions to rosbag2.

    Manages session directories, rosbag2 recording, metadata generation,
    and storage cleanup based on configured limits.
    """

    def __init__(self):
        super().__init__('data_logger')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self.calibration_data_dir = os.path.expanduser(
            self.get_parameter('calibration_data_dir').value
        )
        self.compression = self.get_parameter('compression').value
        self.storage_preset = self.get_parameter('storage_preset').value
        self.record_topics = self.get_parameter('record_topics').value
        self.max_sessions_to_keep = self.get_parameter('max_sessions_to_keep').value
        self.max_storage_gb = self.get_parameter('max_storage_gb').value
        self.min_disk_space_gb = self.get_parameter('min_disk_space_gb').value
        self.robot_id = self.get_parameter('robot_id').value
        self.operator_name = self.get_parameter('operator_name').value
        self.node_version = self.get_parameter('node_version').value
        self.status_publish_rate_hz = self.get_parameter('status_publish_rate_hz').value

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Service servers
        self.start_recording_srv = self.create_service(
            StartRecording,
            '/calibration/start_recording',
            self._start_recording_callback
        )

        self.stop_recording_srv = self.create_service(
            StopRecording,
            '/calibration/stop_recording',
            self._stop_recording_callback
        )

        self.get_status_srv = self.create_service(
            GetRecordingStatus,
            '/calibration/get_recording_status',
            self._get_status_callback
        )

        # Publisher
        self.status_pub = self.create_publisher(
            RecordingStatus,
            '/calibration/recording_status',
            qos_profile
        )

        # Timer for status publishing
        self.status_timer = self.create_timer(
            1.0 / self.status_publish_rate_hz,
            self._status_timer_callback
        )

        # Recording state
        self.is_recording: bool = False
        self.session_id: str = ""
        self.session_dir: Optional[Path] = None
        self.rosbag_dir: Optional[Path] = None
        self.start_time: Optional[datetime] = None
        self.writer: Optional[SequentialWriter] = None
        self.recorded_topics: List[str] = []

        # Create base calibration data directory if it doesn't exist
        try:
            Path(self.calibration_data_dir).mkdir(parents=True, exist_ok=True)
            self.get_logger().info(
                f'Data Logger initialized: data_dir={self.calibration_data_dir}, '
                f'topics={len(self.record_topics)}, max_sessions={self.max_sessions_to_keep}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to create calibration data directory: {e}')

    def _declare_parameters(self):
        """Declare all node parameters with default values."""
        self.declare_parameter('calibration_data_dir', f'{os.path.expanduser("~")}/miro_calibration_data')
        self.declare_parameter('compression', 'zstd')
        self.declare_parameter('storage_preset', 'sqlite3')
        self.declare_parameter('record_topics', [])
        self.declare_parameter('max_sessions_to_keep', 20)
        self.declare_parameter('max_storage_gb', 10.0)
        self.declare_parameter('min_disk_space_gb', 1.0)
        self.declare_parameter('robot_id', 'miro_v2_001')
        self.declare_parameter('operator_name', 'default')
        self.declare_parameter('node_version', '1.0.0')
        self.declare_parameter('status_publish_rate_hz', 1.0)

    def _generate_session_id(self, custom_suffix: str = "") -> str:
        """
        Generate session ID from current timestamp.

        Args:
            custom_suffix: Optional custom suffix to append

        Returns:
            Session ID in format YYYYMMDD_HHMMSS or YYYYMMDD_HHMMSS_suffix
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if custom_suffix:
            return f"{timestamp}_{custom_suffix}"
        return timestamp

    def _check_disk_space(self) -> tuple[bool, float]:
        """
        Check available disk space.

        Returns:
            Tuple of (sufficient_space, available_gb)
        """
        stat = shutil.disk_usage(self.calibration_data_dir)
        available_gb = stat.free / (1024 ** 3)
        sufficient = available_gb >= self.min_disk_space_gb
        return sufficient, available_gb

    def _get_total_storage_size(self) -> float:
        """
        Calculate total size of all session directories.

        Returns:
            Total size in GB
        """
        total_bytes = 0
        base_path = Path(self.calibration_data_dir)

        if not base_path.exists():
            return 0.0

        for session_dir in base_path.iterdir():
            if session_dir.is_dir() and session_dir.name.startswith('session_'):
                for file_path in session_dir.rglob('*'):
                    if file_path.is_file():
                        total_bytes += file_path.stat().st_size

        return total_bytes / (1024 ** 3)

    def _get_session_size(self, session_dir: Path) -> float:
        """
        Calculate size of a specific session directory.

        Args:
            session_dir: Path to session directory

        Returns:
            Size in GB
        """
        total_bytes = 0
        for file_path in session_dir.rglob('*'):
            if file_path.is_file():
                total_bytes += file_path.stat().st_size
        return total_bytes / (1024 ** 3)

    def _cleanup_old_sessions(self):
        """
        Clean up old sessions based on max_sessions_to_keep and max_storage_gb.

        Deletes oldest sessions first until both constraints are satisfied.
        """
        base_path = Path(self.calibration_data_dir)

        if not base_path.exists():
            return

        # Get all session directories sorted by modification time (oldest first)
        session_dirs = []
        for session_dir in base_path.iterdir():
            if session_dir.is_dir() and session_dir.name.startswith('session_'):
                session_dirs.append((session_dir, session_dir.stat().st_mtime))

        session_dirs.sort(key=lambda x: x[1])  # Sort by mtime

        # Check session count limit
        while len(session_dirs) > self.max_sessions_to_keep:
            oldest_dir, _ = session_dirs.pop(0)
            try:
                shutil.rmtree(oldest_dir)
                self.get_logger().info(
                    f'Cleaned up old session (count limit): {oldest_dir.name}'
                )
            except Exception as e:
                self.get_logger().error(
                    f'Failed to remove old session {oldest_dir.name}: {e}'
                )

        # Check storage size limit
        total_size_gb = self._get_total_storage_size()
        while total_size_gb > self.max_storage_gb and len(session_dirs) > 1:
            # Keep at least one session (the current one if recording)
            oldest_dir, _ = session_dirs.pop(0)
            session_size_gb = self._get_session_size(oldest_dir)

            try:
                shutil.rmtree(oldest_dir)
                total_size_gb -= session_size_gb
                self.get_logger().info(
                    f'Cleaned up old session (size limit): {oldest_dir.name} '
                    f'({session_size_gb:.2f}GB freed, {total_size_gb:.2f}GB remaining)'
                )
            except Exception as e:
                self.get_logger().error(
                    f'Failed to remove old session {oldest_dir.name}: {e}'
                )
                break

    def _create_session_metadata(self):
        """Create session_metadata.json file with session information."""
        if self.session_dir is None or self.start_time is None:
            return

        metadata = {
            'session_id': self.session_id,
            'start_timestamp': self.start_time.isoformat(),
            'end_timestamp': None,  # Updated on stop
            'duration_sec': None,   # Updated on stop
            'topics_recorded': self.recorded_topics,
            'robot_id': self.robot_id,
            'operator_name': self.operator_name,
            'node_version': self.node_version,
            'rosbag_path': str(self.rosbag_dir) if self.rosbag_dir else None,
            'compression': self.compression,
            'storage_preset': self.storage_preset,
            'file_size_mb': None    # Updated on stop
        }

        metadata_path = self.session_dir / 'session_metadata.json'
        try:
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            self.get_logger().info(f'Created session metadata: {metadata_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to create session metadata: {e}')

    def _update_session_metadata(self, duration_sec: float, file_size_mb: float):
        """
        Update session_metadata.json with final information.

        Args:
            duration_sec: Total recording duration in seconds
            file_size_mb: Final rosbag file size in MB
        """
        if self.session_dir is None:
            return

        metadata_path = self.session_dir / 'session_metadata.json'

        try:
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)

            metadata['end_timestamp'] = datetime.now().isoformat()
            metadata['duration_sec'] = duration_sec
            metadata['file_size_mb'] = file_size_mb

            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)

            self.get_logger().info(
                f'Updated session metadata: duration={duration_sec:.1f}s, size={file_size_mb:.2f}MB'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to update session metadata: {e}')

    def _get_topic_type(self, topic_name: str) -> Optional[str]:
        """
        Get message type for a topic by querying ROS2 graph.

        Args:
            topic_name: Topic name

        Returns:
            Message type string or None if not found
        """
        topic_list = self.get_topic_names_and_types()

        for topic, types in topic_list:
            if topic == topic_name:
                if len(types) > 0:
                    return types[0]  # Return first type
                break

        return None

    def _initialize_rosbag_writer(self) -> bool:
        """
        Initialize rosbag2 SequentialWriter with configured topics.

        Returns:
            True if initialization successful, False otherwise
        """
        if self.rosbag_dir is None:
            return False

        try:
            # Create storage options
            storage_options = StorageOptions(
                uri=str(self.rosbag_dir),
                storage_id=self.storage_preset
            )

            # Create converter options
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            # Initialize writer
            self.writer = SequentialWriter()
            self.writer.open(storage_options, converter_options)

            # Create topics in rosbag
            self.recorded_topics = []
            for topic_name in self.record_topics:
                topic_type = self._get_topic_type(topic_name)

                if topic_type is None:
                    self.get_logger().warn(
                        f'Topic {topic_name} not available, skipping'
                    )
                    continue

                try:
                    topic_metadata = TopicMetadata(
                        name=topic_name,
                        type=topic_type,
                        serialization_format='cdr'
                    )
                    self.writer.create_topic(topic_metadata)
                    self.recorded_topics.append(topic_name)
                    self.get_logger().info(
                        f'Recording topic: {topic_name} ({topic_type})'
                    )
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to create topic {topic_name} in rosbag: {e}'
                    )

            if len(self.recorded_topics) == 0:
                self.get_logger().error('No topics available to record')
                return False

            self.get_logger().info(
                f'Initialized rosbag writer: {len(self.recorded_topics)} topics'
            )
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to initialize rosbag writer: {e}')
            return False

    def _calculate_rosbag_size(self) -> float:
        """
        Calculate current rosbag file size.

        Returns:
            Size in MB
        """
        if self.rosbag_dir is None or not self.rosbag_dir.exists():
            return 0.0

        total_bytes = 0
        for file_path in self.rosbag_dir.rglob('*.db3'):
            if file_path.is_file():
                total_bytes += file_path.stat().st_size

        return total_bytes / (1024 ** 2)

    def _start_recording_callback(
        self,
        request: StartRecording.Request,
        response: StartRecording.Response
    ) -> StartRecording.Response:
        """
        Handle start recording service request.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response with session info or error
        """
        if self.is_recording:
            response.success = False
            response.message = "Recording already in progress"
            response.session_id = self.session_id
            response.storage_path = str(self.session_dir) if self.session_dir else ""
            return response

        # Check disk space
        sufficient_space, available_gb = self._check_disk_space()
        if not sufficient_space:
            response.success = False
            response.message = (
                f"Insufficient disk space: {available_gb:.2f}GB available, "
                f"minimum {self.min_disk_space_gb}GB required"
            )
            return response

        # Generate session ID
        self.session_id = self._generate_session_id(request.session_name)
        self.start_time = datetime.now()

        # Create session directory structure
        try:
            self.session_dir = Path(self.calibration_data_dir) / f'session_{self.session_id}'
            self.session_dir.mkdir(parents=True, exist_ok=False)

            # Create rosbag subdirectory
            self.rosbag_dir = self.session_dir / f'rosbag2_{self.session_id}'
            self.rosbag_dir.mkdir(parents=True, exist_ok=False)

            # Create tests subdirectory for future use
            tests_dir = self.session_dir / 'tests'
            tests_dir.mkdir(parents=True, exist_ok=True)

            self.get_logger().info(f'Created session directory: {self.session_dir}')

        except Exception as e:
            response.success = False
            response.message = f"Failed to create session directory: {e}"
            self.get_logger().error(response.message)
            # Cleanup partial directory
            if self.session_dir and self.session_dir.exists():
                try:
                    shutil.rmtree(self.session_dir)
                except:
                    pass
            return response

        # Initialize rosbag writer
        if not self._initialize_rosbag_writer():
            response.success = False
            response.message = "Failed to initialize rosbag writer"
            # Cleanup incomplete recording
            if self.session_dir and self.session_dir.exists():
                try:
                    shutil.rmtree(self.session_dir)
                except Exception as e:
                    self.get_logger().error(f'Failed to cleanup incomplete session: {e}')
            return response

        # Create session metadata
        self._create_session_metadata()

        # Set recording state
        self.is_recording = True

        # Cleanup old sessions
        self._cleanup_old_sessions()

        # Success response
        response.success = True
        response.message = f"Recording started successfully with {len(self.recorded_topics)} topics"
        response.session_id = self.session_id
        response.storage_path = str(self.session_dir)

        self.get_logger().info(
            f'Recording started: session_id={self.session_id}, '
            f'topics={len(self.recorded_topics)}'
        )

        return response

    def _stop_recording_callback(
        self,
        request: StopRecording.Request,
        response: StopRecording.Response
    ) -> StopRecording.Response:
        """
        Handle stop recording service request.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response with recording statistics
        """
        if not self.is_recording:
            response.success = False
            response.message = "No recording in progress"
            return response

        try:
            # Calculate duration
            if self.start_time:
                duration_sec = (datetime.now() - self.start_time).total_seconds()
            else:
                duration_sec = 0.0

            # Close rosbag writer
            if self.writer:
                self.writer = None  # SequentialWriter closes automatically on destruction

            # Calculate final file size
            file_size_mb = self._calculate_rosbag_size()

            # Update session metadata
            self._update_session_metadata(duration_sec, file_size_mb)

            # Success response
            response.success = True
            response.message = f"Recording stopped successfully"
            response.file_size_mb = file_size_mb
            response.duration_sec = duration_sec

            self.get_logger().info(
                f'Recording stopped: session_id={self.session_id}, '
                f'duration={duration_sec:.1f}s, size={file_size_mb:.2f}MB'
            )

            # Reset recording state
            self.is_recording = False
            self.session_id = ""
            self.session_dir = None
            self.rosbag_dir = None
            self.start_time = None
            self.recorded_topics = []

        except Exception as e:
            response.success = False
            response.message = f"Error stopping recording: {e}"
            self.get_logger().error(response.message)

        return response

    def _get_status_callback(
        self,
        request: GetRecordingStatus.Request,
        response: GetRecordingStatus.Response
    ) -> GetRecordingStatus.Response:
        """
        Handle get recording status service request.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response with current recording status
        """
        response.status = self._get_current_status()
        return response

    def _get_current_status(self) -> RecordingStatus:
        """
        Get current recording status.

        Returns:
            RecordingStatus message
        """
        status = RecordingStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'

        status.is_recording = self.is_recording
        status.session_id = self.session_id
        status.topics_recorded = self.recorded_topics
        status.storage_path = str(self.session_dir) if self.session_dir else ""

        if self.start_time:
            status.start_time.sec = int(self.start_time.timestamp())
            status.start_time.nanosec = int(
                (self.start_time.timestamp() % 1) * 1e9
            )

        if self.is_recording:
            status.file_size_mb = self._calculate_rosbag_size()
        else:
            status.file_size_mb = 0.0

        return status

    def _status_timer_callback(self):
        """Publish recording status at configured rate."""
        status = self._get_current_status()
        self.status_pub.publish(status)


def main(args=None):
    """Main entry point for data logger node."""
    rclpy.init(args=args)

    node = DataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop recording if in progress
        if node.is_recording:
            node.get_logger().info('Stopping recording on shutdown...')
            if node.writer:
                node.writer = None  # Close writer

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
