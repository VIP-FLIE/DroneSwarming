# header.frame_id is "local_enu". Positions are meters from PX4's local origin.
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
bool position_valid
bool armed
bool offboard
bool landed
bool battery_available
float32 battery_percentage
