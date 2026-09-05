# ElevatorMission — immutable description of one requested elevator transit.
# Published by competition_loop on /competition/elevator/mission.

std_msgs/Header header

# Monotonically assigned by competition_loop. A new mission_id denotes a new
# transit; dynamic execution phase is intentionally reported elsewhere.
uint32 mission_id

# Public semantic elevator identifier from team_scene_info.json.
string elevator_id

# Floor indices use the existing zero-based building-control convention.
int32 source_floor
int32 target_floor
