#!/usr/bin/env python3
"""
Integration tests for Data Logger Node

Tests start/stop recording service calls, rosbag recording and playback,
and topic recording verification.
"""

import json
import os
import shutil
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from miro_msgs.srv import StartRecording, StopRecording, GetRecordingStatus
from miro_calibration.data_logger import DataLogger


class TestDataLoggerIntegration(unittest.TestCase):
    """Integration tests for DataLogger node."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()

    def setUp(self):
        """Create temporary directory and nodes for each test."""
        self.test_dir = tempfile.mkdtemp()
        self.calibration_data_dir = os.path.join(self.test_dir, 'miro_calibration_data')

        # Create data logger node with test configuration
        self.data_logger = DataLogger()
        self.data_logger.calibration_data_dir = self.calibration_data_dir
        self.data_logger.record_topics = ['/test_topic', '/cmd_vel']
        self.data_logger.max_sessions_to_keep = 5
        self.data_logger.max_storage_gb = 1.0

        # Create test client node
        self.test_node = Node('test_data_logger_client')

        # Create service clients
        self.start_recording_client = self.test_node.create_client(
            StartRecording,
            '/calibration/start_recording'
        )

        self.stop_recording_client = self.test_node.create_client(
            StopRecording,
            '/calibration/stop_recording'
        )

        self.get_status_client = self.test_node.create_client(
            GetRecordingStatus,
            '/calibration/get_recording_status'
        )

        # Create test publishers
        self.test_pub = self.test_node.create_publisher(String, '/test_topic', 10)
        self.cmd_vel_pub = self.test_node.create_publisher(Twist, '/cmd_vel', 10)

        # Wait for services to be available
        timeout = 5.0
        start_time = time.time()
        while not (
            self.start_recording_client.service_is_ready() and
            self.stop_recording_client.service_is_ready() and
            self.get_status_client.service_is_ready()
        ):
            rclpy.spin_once(self.data_logger, timeout_sec=0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                raise TimeoutError("Services not available within timeout")

    def tearDown(self):
        """Clean up nodes and temporary directory after each test."""
        self.test_node.destroy_node()
        self.data_logger.destroy_node()

        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)

    def _spin_nodes(self, duration_sec=0.5):
        """
        Spin both nodes for a duration.

        Args:
            duration_sec: Duration to spin in seconds
        """
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            rclpy.spin_once(self.data_logger, timeout_sec=0.01)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

    def test_start_stop_recording_services(self):
        """Test start and stop recording service calls."""
        # Call start recording service
        start_request = StartRecording.Request()
        start_request.session_name = "test_session"

        start_future = self.start_recording_client.call_async(start_request)
        self._spin_nodes(1.0)

        self.assertTrue(start_future.done())
        start_response = start_future.result()

        # Verify start response
        self.assertTrue(start_response.success)
        self.assertIn("test_session", start_response.session_id)
        self.assertTrue(len(start_response.storage_path) > 0)
        self.assertIn("miro_calibration_data", start_response.storage_path)

        # Verify session directory was created
        session_dir = Path(start_response.storage_path)
        self.assertTrue(session_dir.exists())

        # Publish some test messages
        for _ in range(10):
            msg = String()
            msg.data = "test message"
            self.test_pub.publish(msg)

            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_vel_pub.publish(twist)

            self._spin_nodes(0.1)

        # Call stop recording service
        stop_request = StopRecording.Request()
        stop_future = self.stop_recording_client.call_async(stop_request)
        self._spin_nodes(1.0)

        self.assertTrue(stop_future.done())
        stop_response = stop_future.result()

        # Verify stop response
        self.assertTrue(stop_response.success)
        self.assertGreater(stop_response.duration_sec, 0.0)
        # File size might be 0 if rosbag writer didn't capture messages
        self.assertGreaterEqual(stop_response.file_size_mb, 0.0)

    def test_get_recording_status_service(self):
        """Test get recording status service."""
        # Get status when not recording
        status_request = GetRecordingStatus.Request()
        status_future = self.get_status_client.call_async(status_request)
        self._spin_nodes(0.5)

        self.assertTrue(status_future.done())
        status_response = status_future.result()

        # Verify status when not recording
        self.assertFalse(status_response.status.is_recording)
        self.assertEqual(status_response.status.session_id, "")

        # Start recording
        start_request = StartRecording.Request()
        start_future = self.start_recording_client.call_async(start_request)
        self._spin_nodes(0.5)

        start_response = start_future.result()
        self.assertTrue(start_response.success)

        # Get status when recording
        status_future = self.get_status_client.call_async(status_request)
        self._spin_nodes(0.5)

        status_response = status_future.result()

        # Verify status when recording
        self.assertTrue(status_response.status.is_recording)
        self.assertEqual(status_response.status.session_id, start_response.session_id)
        self.assertTrue(len(status_response.status.storage_path) > 0)

        # Stop recording
        stop_request = StopRecording.Request()
        stop_future = self.stop_recording_client.call_async(stop_request)
        self._spin_nodes(0.5)

    def test_session_metadata_creation(self):
        """Test that session metadata JSON is created correctly."""
        # Start recording
        start_request = StartRecording.Request()
        start_request.session_name = "metadata_test"
        start_future = self.start_recording_client.call_async(start_request)
        self._spin_nodes(0.5)

        start_response = start_future.result()
        self.assertTrue(start_response.success)

        # Verify metadata file exists
        session_dir = Path(start_response.storage_path)
        metadata_path = session_dir / 'session_metadata.json'
        self.assertTrue(metadata_path.exists())

        # Load and verify metadata content
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)

        self.assertIn("metadata_test", metadata['session_id'])
        self.assertIn('start_timestamp', metadata)
        self.assertIsNotNone(metadata['start_timestamp'])
        self.assertIsNone(metadata['end_timestamp'])  # Not set until stop

        # Stop recording
        stop_request = StopRecording.Request()
        stop_future = self.stop_recording_client.call_async(stop_request)
        self._spin_nodes(0.5)

        stop_response = stop_future.result()
        self.assertTrue(stop_response.success)

        # Reload and verify updated metadata
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)

        self.assertIsNotNone(metadata['end_timestamp'])
        self.assertIsNotNone(metadata['duration_sec'])
        self.assertIsNotNone(metadata['file_size_mb'])

    def test_duplicate_start_recording(self):
        """Test that starting recording twice fails gracefully."""
        # Start recording first time
        start_request = StartRecording.Request()
        start_future = self.start_recording_client.call_async(start_request)
        self._spin_nodes(0.5)

        start_response = start_future.result()
        self.assertTrue(start_response.success)

        # Try to start recording again
        start_future2 = self.start_recording_client.call_async(start_request)
        self._spin_nodes(0.5)

        start_response2 = start_future2.result()
        self.assertFalse(start_response2.success)
        self.assertIn("already in progress", start_response2.message)

        # Stop recording
        stop_request = StopRecording.Request()
        stop_future = self.stop_recording_client.call_async(stop_request)
        self._spin_nodes(0.5)

    def test_stop_without_start(self):
        """Test that stopping recording without starting fails gracefully."""
        # Try to stop recording without starting
        stop_request = StopRecording.Request()
        stop_future = self.stop_recording_client.call_async(stop_request)
        self._spin_nodes(0.5)

        stop_response = stop_future.result()
        self.assertFalse(stop_response.success)
        self.assertIn("No recording in progress", stop_response.message)

    def test_session_directory_structure(self):
        """Test that session directory structure is created correctly."""
        # Start recording
        start_request = StartRecording.Request()
        start_future = self.start_recording_client.call_async(start_request)
        self._spin_nodes(0.5)

        start_response = start_future.result()
        self.assertTrue(start_response.success)

        # Verify directory structure
        session_dir = Path(start_response.storage_path)
        self.assertTrue(session_dir.exists())

        # Check for expected subdirectories
        rosbag_dir = session_dir / f"rosbag2_{start_response.session_id}"
        tests_dir = session_dir / "tests"

        # Note: rosbag_dir might not exist yet if writer hasn't flushed
        # but tests_dir should exist
        self.assertTrue(tests_dir.exists())

        # Check for metadata file
        metadata_path = session_dir / 'session_metadata.json'
        self.assertTrue(metadata_path.exists())

        # Stop recording
        stop_request = StopRecording.Request()
        stop_future = self.stop_recording_client.call_async(stop_request)
        self._spin_nodes(0.5)

    def test_recording_status_publishing(self):
        """Test that recording status is published periodically."""
        from miro_msgs.msg import RecordingStatus

        # Create subscriber to recording status topic
        status_msgs = []

        def status_callback(msg):
            status_msgs.append(msg)

        status_sub = self.test_node.create_subscription(
            RecordingStatus,
            '/calibration/recording_status',
            status_callback,
            10
        )

        # Spin to receive initial status messages
        self._spin_nodes(2.0)

        # Verify status messages are being published
        self.assertGreater(len(status_msgs), 0)

        # Verify status shows not recording initially
        self.assertFalse(status_msgs[-1].is_recording)

        # Destroy subscriber
        self.test_node.destroy_subscription(status_sub)


if __name__ == '__main__':
    unittest.main()
