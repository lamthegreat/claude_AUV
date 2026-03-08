# Future: DIY sonar scan message
std_msgs/Header header

# Distance to nearest object in meters
float32 range_m

# Bearing in degrees (0 = forward, positive = clockwise)
float32 bearing_deg

# Measurement confidence 0.0-1.0
float32 confidence
