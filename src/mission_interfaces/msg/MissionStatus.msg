std_msgs/Header header

uint8 IDLE=0
uint8 TAKEOFF=1
uint8 EXECUTE=2
uint8 RETURN=3
uint8 LAND=4
uint8 COMPLETE=5
uint8 FAILSAFE=6

uint8 state

bool mission_loaded

bool target_available

string current_cell

uint32 current_waypoint_index

uint32 total_waypoints

float32 progress

float64 distance_to_target

string failure_reason
