# Path execution progress tracking
int32 current_segment    # Current path segment being executed
int32 total_segments     # Total number of segments in path

# Last recorded robot pose
geometry_msgs/Pose last_pose

# Overall path completion percentage (0-100)
float32 completion_percentage
