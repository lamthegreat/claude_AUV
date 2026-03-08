#!/bin/bash
# ESC Calibration Procedure
# Calibrates all 6 ESCs to the same throttle range (1100-1900 μs).
#
# Most hobby ESCs need this once when first used with a new controller.
# Standard calibration: send max throttle → power on ESCs → send min throttle.
#
# DANGER: Remove props/motors from water before calibrating!
#
# Usage: bash scripts/calibrate_escs.sh

echo "=== ESC Calibration Procedure ==="
echo ""
echo "DANGER: This procedure will command maximum throttle briefly."
echo "REMOVE ALL PROPELLERS AND ENSURE MOTORS ARE OUT OF WATER."
echo ""
read -p "Are you CERTAIN the propellers are removed? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted. Remove propellers first."
    exit 1
fi

echo ""
echo "Manual ESC calibration steps:"
echo ""
echo "1. Disconnect ESC power (battery/power supply)"
echo "2. Ensure motor controller firmware is flashed and running"
echo "3. Run the microROS agent:"
echo "   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_motor_ctrl -b 6000000"
echo ""
echo "4. Publish MAX throttle (unarmed flag=0, all pwm=1900):"
echo "   ros2 topic pub --once /auv/thrusters/commands std_msgs/msg/Float32MultiArray \\"
echo "     \"{data: [0.0, 1900.0, 1900.0, 1900.0, 1900.0, 1900.0, 1900.0]}\""
echo ""
echo "5. While publishing, CONNECT ESC POWER"
echo "   Listen for ESC arming beeps (usually 2-3 ascending tones)"
echo ""
echo "6. Publish MIN throttle:"
echo "   ros2 topic pub --once /auv/thrusters/commands std_msgs/msg/Float32MultiArray \\"
echo "     \"{data: [0.0, 1100.0, 1100.0, 1100.0, 1100.0, 1100.0, 1100.0]}\""
echo ""
echo "7. Listen for confirmation beeps, then publish NEUTRAL:"
echo "   ros2 topic pub --once /auv/thrusters/commands std_msgs/msg/Float32MultiArray \\"
echo "     \"{data: [0.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}\""
echo ""
echo "8. ESC calibration complete. Test forward/reverse gently."
echo ""
echo "See docs/commissioning_checklist.md for full commissioning steps."
