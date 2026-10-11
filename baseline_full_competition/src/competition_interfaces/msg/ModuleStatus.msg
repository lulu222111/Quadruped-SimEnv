# ModuleStatus — every competition module reports its state on this message.
# Published on /competition/<module>/status by each functional node.

std_msgs/Header header

# State constants
uint8 UNKNOWN=0
uint8 STARTING=1
uint8 READY=2
uint8 RUNNING=3
uint8 SUCCEEDED=4
uint8 BLOCKED=5
uint8 TIMEOUT=6
uint8 FAILED=7

# Current state of this module
uint8 state

# Whether the module output is considered trustworthy.
# Must remain false when only skeleton/placeholder code is running.
bool valid

# Module-specific confidence [0.0, 1.0]. Meaning defined per module.
float32 confidence

# Human-readable detail string (e.g. "waiting for IMU calibration")
string detail
