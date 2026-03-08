std_msgs/Header header

# PWM pulse width per thruster in microseconds (1100=full reverse, 1500=stop, 1900=full forward)
float32[6] pwm_us

# Normalized thrust command -1.0 to 1.0 (informational; allocator output before PWM scaling)
float32[6] normalized

# Must be true for ESCs to respond; Teensy will safe to 1500us if false
bool armed
