#!/bin/bash
# Flash Teensy 4.1 Sensor Hub firmware using Arduino CLI
# Usage: bash scripts/flash_sensor_hub.sh
#
# Requires arduino-cli to be installed and configured with Teensyduino.
# If using Arduino IDE instead, open firmware/sensor_hub/sensor_hub.ino manually.

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SKETCH="$REPO_ROOT/firmware/sensor_hub/sensor_hub.ino"

echo "=== Flashing Sensor Hub (Teensy 4.1) ==="
echo "Sketch: $SKETCH"
echo ""
echo "NOTE: If arduino-cli is not installed, open the sketch in Arduino IDE:"
echo "  $SKETCH"
echo "  Board: Teensy 4.1"
echo "  USB Type: Serial"
echo "  CPU Speed: 600 MHz"
echo ""

if ! command -v arduino-cli &> /dev/null; then
    echo "arduino-cli not found. Please flash manually via Arduino IDE."
    exit 1
fi

arduino-cli compile \
    --fqbn teensy:avr:teensy41 \
    --build-property "build.extra_flags=-DTEENSY_OPT_FASTER" \
    "$SKETCH"

arduino-cli upload \
    --fqbn teensy:avr:teensy41 \
    --port /dev/auv_sensor_hub \
    "$SKETCH"

echo ""
echo "[✓] Sensor hub flashed successfully."
echo "Run the microROS agent to verify:"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_sensor_hub -b 6000000"
