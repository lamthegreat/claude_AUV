std_msgs/Header header

# Position and orientation in odom frame
geometry_msgs/Pose pose

# Body-frame linear and angular velocities
geometry_msgs/Twist twist

# Depth from depth sensor (ground truth; separate from pose.position.z which is EKF estimate)
float32 depth_m

# Future: altitude above bottom from DVL or sonar
float32 altitude_m

# Control mode constants
uint8 MANUAL=0
uint8 DEPTH_HOLD=1
uint8 HEADING_HOLD=2
uint8 FULL_AUTO=3
uint8 control_mode

bool thrusters_armed
bool imu_calibrated
bool depth_sensor_valid
