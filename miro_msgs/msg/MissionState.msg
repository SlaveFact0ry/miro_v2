# Mission state enumeration
uint8 IDLE = 0
uint8 MAPPING = 1
uint8 READY = 2
uint8 CLEANING = 3
uint8 PAUSED = 4
uint8 COMPLETE = 5
uint8 EMERGENCY_STOP = 6

# Current mission state
uint8 state

# Timestamp of state change
builtin_interfaces/Time timestamp
