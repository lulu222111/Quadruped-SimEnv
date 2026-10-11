# CompetitionState — global competition status published by competition_loop.

std_msgs/Header header

# Competition phase constants
uint8 WAITING=0
uint8 RUNNING=1
uint8 RETURNING=2
uint8 FINISHED=3
uint8 ERROR=4

# Current competition phase
uint8 state

# Elapsed exploration time. Only meaningful in RUNNING / FINISHED.
duration elapsed_time

# Number of hazard candidates accepted so far
uint32 detected_count

# Human-readable detail (e.g. "max_time reached", "finish service called")
string detail
