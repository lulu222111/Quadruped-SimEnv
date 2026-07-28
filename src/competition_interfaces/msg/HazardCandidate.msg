# HazardCandidate — a single detected hazard-source candidate.
# Published by the perception module on /competition/hazard_candidates.
std_msgs/Header header

# Position in the frame specified by header.frame_id.
# Must be converted to "world" frame before result_writer accepts it.
geometry_msgs/Point position

# Semantic label, e.g. "red_sphere", "red_box", "green_sphere"
string label

# Detection confidence [0.0, 1.0]
float32 confidence

# Whether position has been validated and can be written to final result.
# Only candidates with position_valid=true AND frame_id="world" are
# accepted by result_writer.
bool position_valid
