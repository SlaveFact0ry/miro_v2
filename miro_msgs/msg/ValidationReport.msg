# Map validation results
bool passed              # Overall validation result
bool boundary_closed     # Boundary forms a closed polygon

# Pool dimensions (x=length, y=width, z=depth)
geometry_msgs/Point dimensions

# Percentage of map occupied by artifacts/obstacles
float32 artifact_percentage

# Detailed error messages (empty if passed)
string[] error_messages
