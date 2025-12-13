# RPLIDAR Detailed Statistics Message
# Detailed metrics for RPLIDAR sensor health monitoring

# Message header
std_msgs/Header header

# Connection Status
bool connected                  # True if receiving data within timeout
float32 time_since_last_msg     # Time since last scan message in seconds

# Frequency Metrics
float32 rate_hz                 # Measured scan rate in Hz
float32 expected_rate_hz        # Expected scan rate (typically 10Hz)
float32 rate_error_percent      # Deviation from expected rate in percentage

# Scan Quality Metrics
uint32 point_count              # Number of points in latest scan
float32 point_count_avg         # Average point count over window
uint32 min_point_count          # Minimum expected point count
float32 range_min               # Minimum range in latest scan (meters)
float32 range_max               # Maximum range in latest scan (meters)
float32 range_mean              # Mean range in latest scan (meters)
float32 range_variance          # Variance of ranges in latest scan
float32 max_range_variance      # Maximum acceptable range variance

# Status
bool healthy                    # True if all metrics are within thresholds
