# /hub/queue: passenger_detector (M1-A) -> predictor, coordinator, safety. 5 Hz, R10.
# Low confidence or valid=false means UNKNOWN, never zero people (assignment 6.1).
std_msgs/Header header
string zone_id
uint32 count
float32 confidence
bool valid
bool crossing_occupied
