# Safety monitoring flags
bool tilt_ok         # Robot is within safe tilt angle
bool battery_ok      # Battery level is sufficient
bool comms_ok        # Communication with control system is stable

# Reason for safety violation (empty if all OK)
string violation_reason
