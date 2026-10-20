# One slot's physical and administrative state, nested in ParkingState.
# state is FREE, RESERVED, OCCUPIED or UNKNOWN (assignment 6.1).
string slot_id
string state
string vehicle_id
string reservation_id
builtin_interfaces/Time expires_at
