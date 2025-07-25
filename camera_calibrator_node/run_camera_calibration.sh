#!/bin/bash

# Script to build and run ROS camera calibration container with GUI tools

# Set up trap for cleanup on Ctrl+C
trap cleanup INT TERM EXIT

echo "=== Starting ROS Camera Calibration Setup ==="

# Enable X11 forwarding
echo "Enabling X11 forwarding..."
xhost +local:root

# Build the Docker image
echo "Building Docker image..."
docker build -t ros_camera_calibration .

# Clean up dangling images
echo "Cleaning up dangling images..."
docker image prune --force

# Function to wait for container to be ready
wait_for_container() {
    echo "Waiting for container to be ready..."
    local max_attempts=30
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        if docker ps -q | head -n 1 > /dev/null 2>&1; then
            local container_id=$(docker ps -q | head -n 1)
            if [ ! -z "$container_id" ]; then
                # Test if we can execute commands in the container
                if docker exec $container_id echo "test" > /dev/null 2>&1; then
                    echo "Container is ready!"
                    return 0
                fi
            fi
        fi
        echo "Attempt $attempt/$max_attempts - Container not ready yet..."
        sleep 2
        ((attempt++))
    done
    
    echo "Error: Container failed to become ready within expected time"
    exit 1
}

# Array to store terminal PIDs for cleanup
TERMINAL_PIDS=()

# Function to open new terminal tab and run command
open_terminal_tab() {
    local title="$1"
    local command="$2"
    local pid
    
    # Try different terminal emulators
    if command -v gnome-terminal > /dev/null 2>&1; then
        gnome-terminal --tab --title="$title" -- bash -c "$command; exec bash" &
        pid=$!
        TERMINAL_PIDS+=($pid)
    elif command -v konsole > /dev/null 2>&1; then
        konsole --new-tab -e bash -c "$command; exec bash" &
        pid=$!
        TERMINAL_PIDS+=($pid)
    elif command -v xfce4-terminal > /dev/null 2>&1; then
        xfce4-terminal --tab --title="$title" -e "bash -c '$command; exec bash'" &
        pid=$!
        TERMINAL_PIDS+=($pid)
    elif command -v terminator > /dev/null 2>&1; then
        terminator --new-tab --title="$title" -e "bash -c '$command; exec bash'" &
        pid=$!
        TERMINAL_PIDS+=($pid)
    else
        echo "Warning: No supported terminal emulator found. Please run these commands manually:"
        echo "Tab 1 ($title): $command"
        return 1
    fi
}

# Function to cleanup terminals and container
cleanup() {
    echo ""
    echo "Cleaning up..."
    
    # Kill all opened terminal processes
    for pid in "${TERMINAL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo "Closing terminal with PID: $pid"
            kill $pid 2>/dev/null
        fi
    done
    
    # Find and stop any running containers
    local running_container=$(docker ps -q --filter "ancestor=ros_camera_calibration" | head -n 1)
    if [ ! -z "$running_container" ]; then
        echo "Stopping container: $running_container"
        docker stop $running_container 2>/dev/null
    fi
    
    # Clean up X11 forwarding
    xhost -local:root 2>/dev/null
    
    echo "Cleanup complete."
    exit 0
}

# Start the main container in an interactive terminal tab
echo "Starting main container in interactive mode..."
MAIN_CONTAINER_CMD="docker run -it --rm --device=/dev/video0 -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros_camera_calibration bash -c 'source /ros_ws/devel/setup.bash && roslaunch /ros_ws/launch/camera_calibration.launch & echo \"Launch started in background. Press Ctrl+C to stop launch, or type commands to interact with container.\"; bash'"
open_terminal_tab "Main Container" "$MAIN_CONTAINER_CMD"

# Wait for container to start and be ready
echo "Waiting for container to start..."
sleep 8

# Get the container ID (it might take a moment to appear)
echo "Looking for running container..."
max_attempts=15
attempt=1

while [ $attempt -le $max_attempts ]; do
    CONTAINER_ID=$(docker ps -q --filter "ancestor=ros_camera_calibration" | head -n 1)
    if [ ! -z "$CONTAINER_ID" ]; then
        echo "Found container: $CONTAINER_ID"
        break
    fi
    echo "Attempt $attempt/$max_attempts - Looking for container..."
    sleep 2
    ((attempt++))
done

if [ -z "$CONTAINER_ID" ]; then
    echo "Warning: Could not find running container ID. The GUI tabs may not work properly."
    echo "Please check the main container tab and ensure the container is running."
    echo "You can manually get the container ID with: docker ps"
    exit 1
fi

# Wait a bit more for ROS to initialize
echo "Waiting for ROS to initialize..."
sleep 5

# Open terminal tab for rqt_image_view
echo "Opening rqt_image_view tab..."
RQT_IMAGE_CMD="docker exec -it $CONTAINER_ID bash -c 'source /ros_ws/devel/setup.bash && rqt_image_view'"
open_terminal_tab "RQT Image View" "$RQT_IMAGE_CMD"

# Wait a moment before opening the second tab
sleep 2

# Open terminal tab for rqt_service_caller
echo "Opening rqt_service_caller tab..."
RQT_SERVICE_CMD="docker exec -it $CONTAINER_ID bash -c 'source /ros_ws/devel/setup.bash && rqt --standalone rqt_service_caller'"
open_terminal_tab "RQT Service Caller" "$RQT_SERVICE_CMD"

echo ""
echo "=== Setup Complete ==="
echo "Container ID: $CONTAINER_ID"
echo ""
echo "The following should now be running:"
echo "1. Main interactive container tab with launch running in background"
echo "2. Terminal tab with rqt_image_view"
echo "3. Terminal tab with rqt_service_caller"
echo ""
echo "In the main container tab, you can:"
echo "- See launch output and logs"
echo "- Run additional ROS commands (rosnode list, rostopic list, etc.)"
echo "- Call services directly: rosservice call /capture_frame"
echo "- Press Ctrl+C to stop the launch file"
echo ""
echo "To capture a frame, use either:"
echo "- The service caller tab GUI, or"
echo "- The main container tab: rosservice call /capture_frame"
echo ""

# Keep the script running to monitor the main container
echo "Main container is running interactively in its own tab."
echo "Press Ctrl+C in this terminal to stop everything and close all tabs."
echo ""

# Wait for the main container to exit or for user interrupt
while true; do
    # Check if the main container is still running
    running_container=$(docker ps -q --filter "ancestor=ros_camera_calibration" | head -n 1)
    if [ -z "$running_container" ]; then
        echo "Main container has stopped. Cleaning up..."
        cleanup
    fi
    sleep 2
done