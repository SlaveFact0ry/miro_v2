# SpeedMode.msg
# Message for calibration system speed preset indication

# Speed mode enumeration
uint8 SLOW = 0
uint8 MEDIUM = 1
uint8 FAST = 2

# Current speed mode
uint8 mode

# Speed values for this mode (m/s and rad/s)
float32 linear_speed
float32 angular_speed

# Timestamp when mode was set
builtin_interfaces/Time timestamp
