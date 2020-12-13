float32 BATTERY_MIN = 0.0
float32 BATTERY_MAX = 1.0

uint8 IDLE = 0
uint8 MOVING = 1
uint8 CHARGING = 2

std_msgs/Header header
geometry_msgs/Point coords
geometry_msgs/Point pose
float32 battery_level
uint8 state
string state_str
float64 last_home_visit
uint64 wheel_lubrication_counter
robot_msgs/MapMsg internal_map
