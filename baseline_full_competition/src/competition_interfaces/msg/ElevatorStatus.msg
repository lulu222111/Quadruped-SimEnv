# ElevatorStatus — executor phase for one elevator transit.
# Future publisher: door_elevator_executor on /competition/elevator/status.

std_msgs/Header header

# Identity copied from the immutable ElevatorMission.
uint32 mission_id
string elevator_id
int32 source_floor
int32 target_floor

# Transit phase constants. READY_TO_ENTER and READY_TO_EXIT require formal,
# observable evidence; time-based inference and simulator truth are forbidden.
uint8 IDLE=0
uint8 APPROACHING=1
uint8 AT_APPROACH=2
uint8 WAITING_ELEVATOR=3
uint8 READY_TO_ENTER=4
uint8 ENTERING=5
uint8 IN_CABIN=6
uint8 RIDING=7
uint8 READY_TO_EXIT=8
uint8 EXITING=9
uint8 COMPLETE=10
uint8 FAILED=11

# Current transit phase and whether it is backed by valid, current evidence.
uint8 phase
bool valid
string detail
