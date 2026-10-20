# /hub/demand: demand_predictor (M1-B) -> coordinator, dashboard. 1 Hz, R10.
std_msgs/Header header
uint32 horizon_sec
float32 predicted_count
float32 baseline_count
string model_id
bool valid
