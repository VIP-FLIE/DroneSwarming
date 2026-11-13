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
# Revision changes when the local position frame or its geographic reference resets.
uint64 local_frame_revision
# Geographic coordinates of the local_enu origin. Altitude is meters AMSL.
# These fields are usable only when global_reference_valid is true.
bool global_reference_valid
float64 reference_latitude_deg
float64 reference_longitude_deg
float64 reference_altitude_amsl_m
