#!/bin/bash
# Flash Teensy 4.0 Motor Controller firmware using Arduino CLI
# Usage: bash scripts/flash_motor_controller.sh

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SKETCH="$REPO_ROOT/firmware/motor_controller/motor_controller.ino"

echo "=== Flashing Motor Controller (Teensy 4.0) ==="
echo "Sketch: $SKETCH"
echo ""
echo "NOTE: If arduino-cli is not installed, open the sketch in Arduino IDE:"
echo "  $SKETCH"
echo "  Board: Teensy 4.0"
echo "  USB Type: Serial"
echo "  CPU Speed: 600 MHz"
echo ""
echo "WARNING: Disconnect motors/ESCs before flashing!"
read -p "Motors disconnected? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted. Disconnect motors before flashing."
    exit 1
fi

if ! command -v arduino-cli &> /dev/null; then
    echo "arduino-cli not found. Please flash manually via Arduino IDE."
    exit 1
fi

arduino-cli compile \
    --fqbn teensy:avr:teensy40 \
    "$SKETCH"

arduino-cli upload \
    --fqbn teensy:avr:teensy40 \
    --port /dev/auv_motor_ctrl \
    "$SKETCH"

echo ""
echo "[✓] Motor controller flashed successfully."
echo "Verify watchdog by running the agent and NOT publishing commands:"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_motor_ctrl -b 6000000"
echo "  (All ESCs should stay at neutral after 200ms of silence)"
