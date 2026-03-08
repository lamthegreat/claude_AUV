std_msgs/Header header

# Fused orientation quaternion (from BNO085 ARVR Stabilized Rotation Vector)
geometry_msgs/Quaternion orientation

# Angular velocity in rad/s (body frame)
geometry_msgs/Vector3 angular_velocity

# Linear acceleration in m/s^2 (body frame, gravity-compensated)
geometry_msgs/Vector3 linear_acceleration

# Row-major 3x3 covariance matrix for orientation
float64[9] orientation_covariance

# BNO085 calibration status (0=uncalibrated, 3=fully calibrated)
uint8 calibration_system
uint8 calibration_gyro
uint8 calibration_accel
uint8 calibration_mag

# True when all calibration fields == 3
bool fully_calibrated
