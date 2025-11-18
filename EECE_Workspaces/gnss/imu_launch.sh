#!/bin/bash
# ------------------------------------------------------------------
# Script: run_vn_driver.sh
# Purpose: Navigate to GNSS workspace, source setup, and launch vn_driver
# ------------------------------------------------------------------

# Navigate to the workspace directory
cd ~/gitRepos/ROS2/EECE_Workspaces/gnss || {
  echo "Error: Could not find GNSS workspace directory."
  exit 1
}

# Source the workspace setup file
if [ -f install/setup.bash ]; then
  source install/setup.bash
  echo "Workspace environment sourced."
else
  echo "Error: install/setup.bash not found. Please build the workspace first."
  exit 1
fi

# Launch the vn_driver
echo "Launching vn_driver..."
ros2 launch vn_driver launch_and_record.py
