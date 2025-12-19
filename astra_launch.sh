#!/bin/bash 
set -e

### STEP 0 — Environment setup
echo ">>> Sourcing ROS Melodic environment..."
source /opt/ros/melodic/setup.bash
source ~/astra_ws/devel/setup.bash 2>/dev/null || true

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo ""

cleanup() {
    echo ""
    echo "⚠️  Caught Ctrl+C — stopping all ROS processes..."
    pkill -f roscore || true
    pkill -f roslaunch || true
    pkill -f usb_cam_node || true
    pkill -f rqt_image_view || true
    echo "✅ All background ROS nodes terminated."
    exit 0
}
trap cleanup SIGINT SIGTERM

### STEP 1 — Kill any leftover roscore
echo ">>> Checking for existing roscore..."
if pgrep -x "roscore" > /dev/null; then
    echo "Old roscore detected. Killing..."
    pkill -9 roscore || true
    pkill -9 rosmaster || true
    sleep 2
else
    echo "No previous roscore running."
fi
echo ""

### STEP 2 — Start roscore in background
echo ">>> Starting roscore..."
roscore > /tmp/roscore.log 2>&1 &
sleep 5
if ! pgrep -x "roscore" > /dev/null; then
    echo "Failed to start roscore. Check /tmp/roscore.log"
    exit 1
fi
echo "roscore started successfully."
echo ""

echo "=== Step 3: Resetting Astra Pro Plus USB devices ==="
for id in 050f 060f; do
  echo "Resetting Orbbec device $id"
  busdev=$(lsusb | grep "2bc5:$id" | awk '{print "/dev/bus/usb/"$2"/"$4}' | tr -d ":")
  if [ -n "$busdev" ]; then
    sudo /usr/local/bin/usbreset $busdev
  else
    echo "Device 2bc5:$id not found — skipping."
  fi
done
sleep 3

echo "=== Step 4: Launching Astra Pro Plus ==="
roslaunch astra_camera astraproplus.launch &
sleep 5

echo "=== STEP 5 : Running YOLOv5 Human Detection Code ==="
rosrun jetauto_cam yolov5_objectdetection.py &

echo "=== STEP 6 : Running UR Robot code ==="
/usr/bin/env python3 /ur_rtde/ur_controller.py &

echo ""
echo "All systems started successfully!"
echo "Press Ctrl+C to stop all ROS nodes."
echo ""

# Keep script alive until Ctrl+C is pressed
while true; do
    sleep 1
done
