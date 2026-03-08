std_msgs/Header header

# Depth below surface in meters (positive = deeper)
float32 depth_m

# Raw pressure reading in Pascals
float32 pressure_pa

# Water/sensor temperature in Celsius
float32 temperature_c

# Depth measurement variance (m^2); used by EKF
float32 depth_variance
