# IMU Detailed Statistics Message
# Detailed metrics for IMU sensor health monitoring

# Message header
std_msgs/Header header

# Connection Status
bool connected                  # True if receiving data within timeout
float32 time_since_last_msg     # Time since last IMU message in seconds

# Frequency Metrics
float32 rate_hz                 # Measured IMU rate in Hz
float32 expected_rate_hz        # Expected IMU rate (typically 20Hz)
float32 rate_error_percent      # Deviation from expected rate in percentage

# Gyroscope Metrics
float32 gyro_x                  # Gyroscope x-axis (rad/s)
float32 gyro_y                  # Gyroscope y-axis (rad/s)
float32 gyro_z                  # Gyroscope z-axis (rad/s)
float32 gyro_bias_z             # Estimated z-axis bias when stationary (rad/s)
float32 max_gyro_bias           # Maximum acceptable gyro bias

# Accelerometer Metrics
float32 accel_x                 # Acceleration x-axis (m/s²)
float32 accel_y                 # Acceleration y-axis (m/s²)
float32 accel_z                 # Acceleration z-axis (m/s²)
float32 accel_magnitude         # Acceleration magnitude (m/s²)
float32 expected_accel_magnitude # Expected magnitude (typically 9.81 m/s²)
float32 accel_error             # Deviation from expected magnitude (m/s²)
float32 max_accel_noise         # Maximum acceptable acceleration noise

# Status
bool healthy                    # True if all metrics are within thresholds
