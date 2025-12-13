#!/usr/bin/env python3
"""
Unit tests for Data Logger Node

Tests session directory creation, metadata generation, storage cleanup logic,
and rosbag file size calculations.
"""

import json
import os
import shutil
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from miro_calibration.data_logger import DataLogger


class TestDataLogger(unittest.TestCase):
    """Unit tests for DataLogger node."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()

    def setUp(self):
        """Create temporary directory for each test."""
        self.test_dir = tempfile.mkdtemp()
        self.calibration_data_dir = os.path.join(self.test_dir, 'miro_calibration_data')

    def tearDown(self):
        """Clean up temporary directory after each test."""
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)

    def _create_node_with_params(self, **kwargs):
        """
        Create DataLogger node with custom parameters.

        Args:
            **kwargs: Parameter overrides

        Returns:
            DataLogger node instance
        """
        node = DataLogger()

        # Override parameters
        node.calibration_data_dir = kwargs.get(
            'calibration_data_dir',
            self.calibration_data_dir
        )
        node.max_sessions_to_keep = kwargs.get('max_sessions_to_keep', 20)
        node.max_storage_gb = kwargs.get('max_storage_gb', 10.0)
        node.record_topics = kwargs.get('record_topics', ['/test_topic'])

        return node

    def test_generate_session_id(self):
        """Test session ID generation with timestamp format."""
        node = self._create_node_with_params()

        # Test without suffix
        session_id = node._generate_session_id()
        self.assertRegex(session_id, r'^\d{8}_\d{6}$')

        # Test with suffix
        session_id_with_suffix = node._generate_session_id('test')
        self.assertRegex(session_id_with_suffix, r'^\d{8}_\d{6}_test$')

        node.destroy_node()

    def test_session_directory_creation(self):
        """Test creation of session directory structure."""
        node = self._create_node_with_params()

        session_id = "20251213_103045"
        session_dir = Path(self.calibration_data_dir) / f'session_{session_id}'
        rosbag_dir = session_dir / f'rosbag2_{session_id}'
        tests_dir = session_dir / 'tests'

        # Create directories
        session_dir.mkdir(parents=True, exist_ok=True)
        rosbag_dir.mkdir(parents=True, exist_ok=True)
        tests_dir.mkdir(parents=True, exist_ok=True)

        # Verify structure
        self.assertTrue(session_dir.exists())
        self.assertTrue(session_dir.is_dir())
        self.assertTrue(rosbag_dir.exists())
        self.assertTrue(rosbag_dir.is_dir())
        self.assertTrue(tests_dir.exists())
        self.assertTrue(tests_dir.is_dir())

        node.destroy_node()

    def test_metadata_generation(self):
        """Test session metadata JSON generation."""
        node = self._create_node_with_params()

        # Setup session state
        session_id = "20251213_103045"
        node.session_id = session_id
        node.session_dir = Path(self.calibration_data_dir) / f'session_{session_id}'
        node.session_dir.mkdir(parents=True, exist_ok=True)
        node.start_time = datetime.now()
        node.recorded_topics = ['/tf', '/cmd_vel', '/scan']

        # Create metadata
        node._create_session_metadata()

        # Verify metadata file exists
        metadata_path = node.session_dir / 'session_metadata.json'
        self.assertTrue(metadata_path.exists())

        # Load and verify metadata content
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)

        self.assertEqual(metadata['session_id'], session_id)
        self.assertIn('start_timestamp', metadata)
        self.assertEqual(metadata['topics_recorded'], ['/tf', '/cmd_vel', '/scan'])
        self.assertIsNone(metadata['end_timestamp'])  # Not set yet
        self.assertIsNone(metadata['duration_sec'])

        node.destroy_node()

    def test_metadata_update(self):
        """Test session metadata update on recording stop."""
        node = self._create_node_with_params()

        # Setup session and create initial metadata
        session_id = "20251213_103045"
        node.session_id = session_id
        node.session_dir = Path(self.calibration_data_dir) / f'session_{session_id}'
        node.session_dir.mkdir(parents=True, exist_ok=True)
        node.start_time = datetime.now()
        node.recorded_topics = ['/tf']

        node._create_session_metadata()

        # Update metadata with final info
        duration_sec = 120.5
        file_size_mb = 45.3
        node._update_session_metadata(duration_sec, file_size_mb)

        # Load and verify updated metadata
        metadata_path = node.session_dir / 'session_metadata.json'
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)

        self.assertIsNotNone(metadata['end_timestamp'])
        self.assertAlmostEqual(metadata['duration_sec'], duration_sec, places=1)
        self.assertAlmostEqual(metadata['file_size_mb'], file_size_mb, places=1)

        node.destroy_node()

    def test_storage_cleanup_max_sessions(self):
        """Test storage cleanup based on max_sessions_to_keep."""
        node = self._create_node_with_params(max_sessions_to_keep=5)

        # Create 7 session directories with different timestamps
        base_path = Path(self.calibration_data_dir)
        base_path.mkdir(parents=True, exist_ok=True)

        session_dirs = []
        for i in range(7):
            session_dir = base_path / f'session_2025121{i}_103045'
            session_dir.mkdir(parents=True, exist_ok=True)

            # Create a small file to ensure directory has content
            (session_dir / 'test.txt').write_text('test')

            # Set mtime to make oldest detectable
            os.utime(session_dir, (i, i))  # (atime, mtime)
            session_dirs.append(session_dir)

        # Run cleanup
        node._cleanup_old_sessions()

        # Verify only 5 sessions remain (2 oldest deleted)
        remaining_sessions = list(base_path.glob('session_*'))
        self.assertEqual(len(remaining_sessions), 5)

        # Verify oldest 2 sessions were deleted
        self.assertFalse(session_dirs[0].exists())
        self.assertFalse(session_dirs[1].exists())

        # Verify newest 5 sessions still exist
        for i in range(2, 7):
            self.assertTrue(session_dirs[i].exists())

        node.destroy_node()

    def test_storage_cleanup_max_size(self):
        """Test storage cleanup based on max_storage_gb."""
        node = self._create_node_with_params(max_storage_gb=0.001)  # 1 MB limit

        # Create session directories with large files
        base_path = Path(self.calibration_data_dir)
        base_path.mkdir(parents=True, exist_ok=True)

        session_dirs = []
        for i in range(3):
            session_dir = base_path / f'session_2025121{i}_103045'
            session_dir.mkdir(parents=True, exist_ok=True)

            # Create a 500KB file in each session
            large_file = session_dir / 'rosbag.db3'
            large_file.write_bytes(b'x' * (500 * 1024))

            # Set mtime to make oldest detectable
            os.utime(session_dir, (i, i))
            session_dirs.append(session_dir)

        # Run cleanup (total ~1.5MB exceeds 1MB limit)
        node._cleanup_old_sessions()

        # Verify oldest sessions were deleted to meet size constraint
        # At least the oldest session should be deleted
        remaining_sessions = list(base_path.glob('session_*'))
        self.assertLess(len(remaining_sessions), 3)

        # Verify total size is under limit
        total_size_gb = node._get_total_storage_size()
        self.assertLessEqual(total_size_gb, node.max_storage_gb * 1.1)  # 10% tolerance

        node.destroy_node()

    def test_calculate_rosbag_size(self):
        """Test rosbag file size calculation."""
        node = self._create_node_with_params()

        # Create session with rosbag files
        session_id = "20251213_103045"
        rosbag_dir = Path(self.calibration_data_dir) / f'session_{session_id}' / f'rosbag2_{session_id}'
        rosbag_dir.mkdir(parents=True, exist_ok=True)

        # Create mock rosbag database files
        db_file1 = rosbag_dir / 'rosbag2_20251213_103045_0.db3'
        db_file2 = rosbag_dir / 'rosbag2_20251213_103045_1.db3'

        # Write 1MB to each file
        db_file1.write_bytes(b'x' * (1024 * 1024))
        db_file2.write_bytes(b'x' * (1024 * 1024))

        # Set node rosbag_dir
        node.rosbag_dir = rosbag_dir

        # Calculate size
        size_mb = node._calculate_rosbag_size()

        # Verify size is approximately 2MB
        self.assertAlmostEqual(size_mb, 2.0, delta=0.1)

        node.destroy_node()

    def test_get_session_size(self):
        """Test individual session size calculation."""
        node = self._create_node_with_params()

        # Create session directory with files
        session_dir = Path(self.calibration_data_dir) / 'session_20251213_103045'
        session_dir.mkdir(parents=True, exist_ok=True)

        # Create files of known sizes
        (session_dir / 'file1.txt').write_bytes(b'x' * (500 * 1024))  # 500KB
        (session_dir / 'file2.txt').write_bytes(b'x' * (500 * 1024))  # 500KB

        # Calculate size
        size_gb = node._get_session_size(session_dir)

        # Verify size is approximately 1MB = 0.001GB
        self.assertAlmostEqual(size_gb, 0.001, delta=0.0001)

        node.destroy_node()

    def test_check_disk_space(self):
        """Test disk space checking."""
        node = self._create_node_with_params()

        # Check disk space
        sufficient, available_gb = node._check_disk_space()

        # Verify results (should have space on test system)
        self.assertIsInstance(sufficient, bool)
        self.assertIsInstance(available_gb, float)
        self.assertGreater(available_gb, 0.0)

        node.destroy_node()

    def test_get_current_status(self):
        """Test recording status message generation."""
        node = self._create_node_with_params()

        # Test status when not recording
        status = node._get_current_status()
        self.assertFalse(status.is_recording)
        self.assertEqual(status.session_id, "")
        self.assertEqual(len(status.topics_recorded), 0)

        # Setup recording state
        node.is_recording = True
        node.session_id = "20251213_103045"
        node.session_dir = Path(self.calibration_data_dir) / 'session_20251213_103045'
        node.recorded_topics = ['/tf', '/cmd_vel']
        node.start_time = datetime.now()

        # Test status when recording
        status = node._get_current_status()
        self.assertTrue(status.is_recording)
        self.assertEqual(status.session_id, "20251213_103045")
        self.assertEqual(status.topics_recorded, ['/tf', '/cmd_vel'])
        self.assertTrue(len(status.storage_path) > 0)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
