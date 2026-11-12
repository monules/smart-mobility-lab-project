# /hub/safety_decision: safety_supervisor (M5) -> metrics_logger and evaluators. 20 Hz, R100.
# Team extension (not in the assignment; added after the midterm review): one record per
# supervisor decision. The latch state and the final commands decided together travel in
# ONE message, so an evaluator never has to match /hub/safety against /robot/cmd_vel
# across topics (DDS gives no ordering between topics). Read-only evidence: no node may
# use it to decide motion.
std_msgs/Header header     # stamp = time of the decision (supervisor clock)
uint64 seq                 # +1 per decision since the supervisor started; a gap = lost records
bool stop_latched
bool motion_permitted
string reason
string[] agents            # same order as linear_x and angular_z
float32[] linear_x         # final command published for each agent in this decision
float32[] angular_z
