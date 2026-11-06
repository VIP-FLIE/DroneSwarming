# header.frame_id is "local_enu". Positions are meters from PX4's local origin.
# yaw_rad is counterclockwise from east in radians, wrapped to [-pi, pi].
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
bool position_valid
bool velocity_valid
float32 yaw_rad
bool yaw_valid
bool armed
bool offboard
bool landed
bool battery_available
float32 battery_percentage
uint64 setpoint_publication_count
