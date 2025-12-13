# RecordingStatus.msg
# Recording status information for data logger

std_msgs/Header header

bool is_recording                # True if currently recording
string session_id                # Session ID (format: YYYYMMDD_HHMMSS)
builtin_interfaces/Time start_time     # Recording start timestamp
string[] topics_recorded         # List of topics being recorded
string storage_path              # Full path to session directory
float32 file_size_mb             # Current rosbag file size in MB
