#!/usr/bin/env bash

# Test script for joint movement using grpcurl
# Usage: ./test_joint_movement.sh "joint1 joint2 joint3" steps_per_cycle

set -e

# Default values
DEFAULT_HOST="localhost:5000"
DEFAULT_FREQUENCY=100.0

# Parse command line arguments
if [ $# -lt 2 ]; then
    echo "Usage: $0 \"joint1 joint2 joint3\" steps_per_cycle [host:port]"
    echo "Example: $0 \"shoulder_pitch elbow_flex\" 20 localhost:5000"
    exit 1
fi

JOINT_NAMES="$1"
FREQUENCY=$2
HOST=${3:-$DEFAULT_HOST}

echo "Testing joint movement for: ${JOINT_NAMES}"
echo "Frequency: ${FREQUENCY} steps per cycle"
echo "Host: ${HOST}"
echo ""

# Function to check if grpcurl is installed
check_grpcurl() {
    if ! command -v grpcurl &> /dev/null; then
        echo "Error: grpcurl is not installed. Please install it first."
        echo "Installation: go install github.com/fullstorydev/grpcurl/cmd/grpcurl@latest"
        exit 1
    fi
}

# Function to get joint limits
get_joint_limits() {
    echo "Getting joint limits from robot..."
    
    # Get robot info
    RESPONSE=$(grpcurl -plaintext -d '{}' "${HOST}" rosbot_api.RobotApiService/GetRobotInfo)
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to get robot info from ${HOST}"
        exit 1
    fi
    
    echo "Robot info received successfully"
    echo "Available joint names:"
    echo "$RESPONSE" | grep -o '"[^"]*"' | grep -v "project_name" | grep -v "joint_names" | grep -v "custom_joint_names" | sort | uniq
    echo ""
    echo "Response preview:"
    echo "$RESPONSE" | head -20
    echo ""
}

# Function to validate that all specified joints exist
validate_joints() {
    echo "Validating specified joints..."
    
    local missing_joints=""
    
    for joint_name in $JOINT_NAMES; do
        # Check if joint exists in regular joints
        local found=false
        
        # Extract joint names from response
        local available_joints=$(echo "$RESPONSE" | grep -o '"[^"]*"' | grep -v "project_name" | grep -v "joint_names" | grep -v "custom_joint_names" | sort | uniq)
        
        # Check if joint is in available joints
        if echo "$available_joints" | grep -q "\"${joint_name}\""; then
            found=true
        fi
        
        if [ "$found" = false ]; then
            if [ -z "$missing_joints" ]; then
                missing_joints="$joint_name"
            else
                missing_joints="$missing_joints, $joint_name"
            fi
        fi
    done
    
    if [ -n "$missing_joints" ]; then
        echo "Error: The following joints were not found: $missing_joints"
        echo "Available joints:"
        echo "$available_joints"
        exit 1
    fi
    
    echo "All specified joints are valid."
    echo ""
}

# Function to extract joint limits for specific joints
extract_joint_limits() {
    local joint_name=$1
    
    # Extract limits using jq (if available) or basic parsing
    if command -v jq &> /dev/null; then
        # Using jq for better JSON parsing
        LOWER=$(echo "$RESPONSE" | jq -r ".joint_limits.\"${joint_name}\".lower // .custom_joint_limits.\"${joint_name}\".lower" 2>/dev/null)
        UPPER=$(echo "$RESPONSE" | jq -r ".joint_limits.\"${joint_name}\".upper // .custom_joint_limits.\"${joint_name}\".upper" 2>/dev/null)
    else
        # Basic parsing without jq - look for the joint limits section
        LOWER=$(echo "$RESPONSE" | sed -n "/\"${joint_name}\":/,/}/p" | grep "lower" | grep -o "[0-9.-]*")
        UPPER=$(echo "$RESPONSE" | sed -n "/\"${joint_name}\":/,/}/p" | grep "upper" | grep -o "[0-9.-]*")
    fi
    
    if [ -z "$LOWER" ] || [ -z "$UPPER" ]; then
        echo "Warning: Could not extract limits for joint '${joint_name}'"
        echo "Using default limits: -3.14 to 3.14"
        LOWER="-3.14"
        UPPER="3.14"
    fi
    
    echo "Joint '${joint_name}': limits [${LOWER}, ${UPPER}]"
}

# Function to extract all joint limits once and store them
extract_all_joint_limits() {
    echo "Extracting joint limits for all specified joints..."
    
    # Initialize associative arrays for joint limits
    declare -gA JOINT_LIMITS_LOWER
    declare -gA JOINT_LIMITS_UPPER
    
    for joint_name in $JOINT_NAMES; do
        # Extract limits using jq (if available) or basic parsing
        if command -v jq &> /dev/null; then
            # Using jq for better JSON parsing
            LOWER=$(echo "$RESPONSE" | jq -r ".joint_limits.\"${joint_name}\".lower // .custom_joint_limits.\"${joint_name}\".lower" 2>/dev/null)
            UPPER=$(echo "$RESPONSE" | jq -r ".joint_limits.\"${joint_name}\".upper // .custom_joint_limits.\"${joint_name}\".upper" 2>/dev/null)
        else
            # Basic parsing without jq - look for the joint limits section
            LOWER=$(echo "$RESPONSE" | sed -n "/\"${joint_name}\":/,/}/p" | grep "lower" | grep -o "[0-9.-]*")
            UPPER=$(echo "$RESPONSE" | sed -n "/\"${joint_name}\":/,/}/p" | grep "upper" | grep -o "[0-9.-]*")
        fi
        
        if [ -z "$LOWER" ] || [ -z "$UPPER" ]; then
            echo "Warning: Could not extract limits for joint '${joint_name}'"
            echo "Using default limits: -3.14 to 3.14"
            LOWER="-3.14"
            UPPER="3.14"
        fi
        
        # Store limits in associative arrays
        JOINT_LIMITS_LOWER["$joint_name"]="$LOWER"
        JOINT_LIMITS_UPPER["$joint_name"]="$UPPER"
        
        echo "Joint '${joint_name}': limits [${LOWER}, ${UPPER}]"
    done
    
    echo ""
}



# Function to move all joints simultaneously
move_all_joints_simultaneously() {
    echo "Moving all joints simultaneously through their ranges..."
    echo "Frequency: ${FREQUENCY} cycles per second"
    
    local steps=$FREQUENCY  # Number of steps to move through range
    
    # Calculate time per step to complete one cycle in 1/frequency seconds
    # One complete cycle = forward motion (steps+1) + backward motion (steps+1) = 2*(steps+1) total steps
    local cycle_time=$(echo "scale=6; 1 / $FREQUENCY" | bc -l)
    local total_steps_per_cycle=$(echo "2 * ($steps + 1)" | bc -l)
    local step_time=$(echo "scale=6; $cycle_time / $total_steps_per_cycle" | bc -l)
    
    echo "  Cycle time: ${cycle_time}s, Total steps per cycle: ${total_steps_per_cycle}, Step time: ${step_time}s"
    echo "  Press Ctrl+C to stop continuous movement"
    
    # Continuous loop through range of motion (lower → upper → lower)
    while true; do
        # Forward motion: lower to upper
        for step in $(seq 0 $steps); do
            local joint_updates=""
            local first=true
            
            for joint_name in $JOINT_NAMES; do
                # Get pre-extracted limits for this joint
                local LOWER="${JOINT_LIMITS_LOWER[$joint_name]}"
                local UPPER="${JOINT_LIMITS_UPPER[$joint_name]}"
                
                # Calculate position for this step (lower to upper)
                local range=$(echo "$UPPER - $LOWER" | bc -l)
                local step_size=$(echo "scale=6; $range / $steps" | bc -l)
                local position=$(echo "scale=6; $LOWER + ($step * $step_size)" | bc -l)
                # Format position to ensure proper JSON format (no leading dot for decimals)
                position=$(printf "%.6f" "$position")
                
                if [ "$first" = true ]; then
                    joint_updates="\"${joint_name}\": ${position}"
                    first=false
                else
                    joint_updates="${joint_updates}, \"${joint_name}\": ${position}"
                fi
            done
            
            local payload="{\"jointUpdates\": {${joint_updates}}}"
            
            echo "Forward Step $step/$steps: ${joint_updates}"
            
            # Send update
            grpcurl -plaintext -d "$payload" "${HOST}" rosbot_api.RobotApiService/UpdateJoints > /dev/null
            
            if [ $? -ne 0 ]; then
                echo "Error: Failed to update joints at step $step"
            fi
            
            # Wait for next step
            sleep $step_time
        done
        
        # Backward motion: upper to lower
        for step in $(seq $steps -1 0); do
            local joint_updates=""
            local first=true
            
            for joint_name in $JOINT_NAMES; do
                # Get pre-extracted limits for this joint
                local LOWER="${JOINT_LIMITS_LOWER[$joint_name]}"
                local UPPER="${JOINT_LIMITS_UPPER[$joint_name]}"
                
                # Calculate position for this step (upper to lower)
                local range=$(echo "$UPPER - $LOWER" | bc -l)
                local step_size=$(echo "scale=6; $range / $steps" | bc -l)
                local position=$(echo "scale=6; $LOWER + ($step * $step_size)" | bc -l)
                # Format position to ensure proper JSON format (no leading dot for decimals)
                position=$(printf "%.6f" "$position")
                
                if [ "$first" = true ]; then
                    joint_updates="\"${joint_name}\": ${position}"
                    first=false
                else
                    joint_updates="${joint_updates}, \"${joint_name}\": ${position}"
                fi
            done
            
            local payload="{\"jointUpdates\": {${joint_updates}}}"
            
            echo "Backward Step $step/$steps: ${joint_updates}"
            
            # Send update
            grpcurl -plaintext -d "$payload" "${HOST}" rosbot_api.RobotApiService/UpdateJoints > /dev/null
            
            if [ $? -ne 0 ]; then
                echo "Error: Failed to update joints at step $step"
            fi
            
            # Wait for next step
            sleep $step_time
        done
        
        echo "  Completed one back-and-forth cycle, starting next cycle..."
    done
}

# Main execution
main() {
    echo "=== Joint Movement Test Script ==="
    echo ""
    
    # Check dependencies
    check_grpcurl
    
    # Get joint limits
    get_joint_limits
    
    # Validate that all specified joints exist
    validate_joints
    
    # Extract all joint limits once
    extract_all_joint_limits
    
    # Move all joints simultaneously
    echo "Moving joints simultaneously..."
    move_all_joints_simultaneously
    
    echo "=== Test completed ==="
}

# Run main function
main "$@"
