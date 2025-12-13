# Sensor Health Status Message
# Overall sensor health and metrics for RPLIDAR and IMU

# Message header
std_msgs/Header header

# RPLIDAR Status
bool rplidar_connected          # True if receiving scan data within timeout
float32 rplidar_rate_hz         # Actual scan rate in Hz
float32 rplidar_point_count_avg # Average number of points per scan
float32 rplidar_range_variance  # Variance of range measurements

# IMU Status
bool imu_connected              # True if receiving IMU data within timeout
float32 imu_rate_hz             # Actual IMU data rate in Hz
float32 imu_gyro_bias_z         # Gyroscope bias on z-axis (yaw) in rad/s
float32 imu_accel_magnitude     # Acceleration magnitude in m/s²

# Warnings
string[] warnings               # Array of warning messages for issues detected
