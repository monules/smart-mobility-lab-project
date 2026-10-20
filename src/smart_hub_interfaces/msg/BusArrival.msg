# /hub/bus_arrival: bus_manager (M2-A) -> coordinator, display. 2 Hz, R10.
# eta_sec = -1 when eta_valid is false (assignment 6.1).
std_msgs/Header header
string bus_id
string state
float32 eta_sec
bool eta_valid
bool door_open
