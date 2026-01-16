#!/bin/bash

# Quadruped Balance Controller Launch Script
# This script helps launch the ROS2 node and GUI

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Quadruped Balance Controller Launch Script ===${NC}"

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 not found. Please install ROS2 first.${NC}"
    exit 1
fi

# Check for Python dependencies
echo -e "${YELLOW}Checking Python dependencies...${NC}"
python3 -c "import tkinter, matplotlib, numpy, rclpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Missing Python dependencies.${NC}"
    echo "Please install required packages:"
    echo "pip3 install matplotlib numpy"
    echo "sudo apt install python3-tk"
    exit 1
fi

# Source ROS2 setup if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}Sourced ROS2 Humble${NC}"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo -e "${GREEN}Sourced ROS2 Foxy${NC}"
else
    echo -e "${YELLOW}Warning: ROS2 setup not found in standard locations${NC}"
fi

# Function to launch controller node
launch_controller() {
    echo -e "${GREEN}Launching Quadruped Balance Controller Node...${NC}"
    python3 quadruped_balance_controller_pid.py &
    CONTROLLER_PID=$!
    echo "Controller PID: $CONTROLLER_PID"
}

# Function to launch GUI
launch_gui() {
    echo -e "${GREEN}Launching GUI...${NC}"
    sleep 2  # Wait for controller to initialize
    python3 quadruped_gui.py &
    GUI_PID=$!
    echo "GUI PID: $GUI_PID"
}

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}Shutting down...${NC}"
    if [ ! -z "$CONTROLLER_PID" ]; then
        kill $CONTROLLER_PID 2>/dev/null
        echo "Stopped controller node"
    fi
    if [ ! -z "$GUI_PID" ]; then
        kill $GUI_PID 2>/dev/null
        echo "Stopped GUI"
    fi
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Main menu
echo -e "\n${YELLOW}Select launch option:${NC}"
echo "1) Launch Controller Node only"
echo "2) Launch GUI only"  
echo "3) Launch Both (Recommended)"
echo "4) Exit"

read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        launch_controller
        echo -e "${GREEN}Controller running. Press Ctrl+C to stop.${NC}"
        wait $CONTROLLER_PID
        ;;
    2)
        launch_gui
        echo -e "${GREEN}GUI running. Press Ctrl+C to stop.${NC}"
        wait $GUI_PID
        ;;
    3)
        launch_controller
        launch_gui
        echo -e "${GREEN}Both components running. Press Ctrl+C to stop.${NC}"
        # Wait for both processes
        wait
        ;;
    4)
        echo -e "${GREEN}Goodbye!${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}Invalid choice. Exiting.${NC}"
        exit 1
        ;;
esac
