# /hub/events: all decision nodes -> metrics_logger. On change, R100.
# Events report transitions, not every control tick (assignment 6.1).
std_msgs/Header header
string run_id
string scenario_id
string entity_id
string event
string reason
string correlation_id
