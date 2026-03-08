std_msgs/Header header

# Per-thruster current draw in amps (0 if ESC telemetry not available)
float32[6] current_amps

# Per-thruster RPM (0 if ESC telemetry not available)
float32[6] rpm

# Per-thruster status: 0=OK, 1=WARN, 2=ERROR
uint8[6] status

# Milliseconds since last command received (watchdog tracking)
uint32 ms_since_last_cmd
