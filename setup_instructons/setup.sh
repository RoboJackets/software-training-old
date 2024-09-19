#!/bin/bash

##############
# Repo Setup #
##############

# Check if Git is installed
if ! command -v git &> /dev/null; then
    echo "Git is not installed. Please install Git and try again."
    exit 1
fi

# Function to clone repository
clone_repository() {
    local repository_url="$1"
    local destination_directory="$2"
    if [[ -z "$(ls -A "$destination_directory")" ]]; then
        if ! git clone "$repository_url" "$destination_directory"; then
            echo "Failed to clone the repository."
            exit 1
        fi
    else
        echo "Destination directory '$destination_directory' likely contains repo already (not empty). Skipping cloning."
    fi
}

# Determine the platform
platform=$(uname)
if [[ "$platform" == "Darwin" || "$platform" == "Linux" || "$platform" == *"MINGW"* || "$platform" == *"MSYS"* ]]; then
    clone_repository "https://github.com/RoboJackets/software-training.git" "training_ws/src/trainingv2"
    clone_repository "https://github.com/RoboJackets/software-training-old.git" "training_ws/src/software-training-old"
else
    echo "Unsupported platform: $platform"
    exit 1
fi


#################
# Docker Setup  #
#################

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Set up image name + tag
image_name="tiryoh/ros2-desktop-vnc"
image_tag="humble"

# Function to get the current working directory
get_cwd() {
    if [[ "$(uname -s)" == "Darwin" || "$(uname -s)" == "Linux" ]]; then
        echo "$(pwd)"
    elif [[ "$(uname -s)" == *"MINGW"* || "$(uname -s)" == *"MSYS"* ]]; then
        dir="$(dirname "$(readlink -f "$BASH_SOURCE")" | sed 's/^[A-Za-z]://')"
        echo "/$(echo "$dir" | sed 's/\\/\//g' | sed 's/://')"
    else
        echo "Unsupported platform: $(uname -s)"
        exit 1
    fi
}

# Set up container name + location
container_name="rj_training_container"
mount_dir=$(get_cwd)

# Function to start the container
start_container() {
    # Check if the container is already running
    if docker ps -q --filter "name=$container_name" | grep -q .; then
        echo "Container $container_name is already running."
        exit 1
    fi

    # Check if the container already exists
    existing_container=$(docker ps -aq --filter name="$container_name")
    if [[ -n "$existing_container" ]]; then
        # Start the existing container
        docker start "$container_name" >/dev/null 2>&1
        echo "Container $container_name started."
    else
        # Run the container and mount with additional options on first creation
        docker pull --platform=linux/amd64 "$image_name:$image_tag"
        docker run -p 6060:80 --shm-size=512m --security-opt seccomp=unconfined -d --name "$container_name" -v "$mount_dir:/home/ubuntu/rj_training_container" "$image_name:$image_tag"
        echo "Container $container_name pulled, created, and started"
    fi
}

# Function to stop the container
stop_container() {
    # Stop the container
    docker stop "$container_name" >/dev/null 2>&1
    echo "Container $container_name stopped"
}

# Function to display usage instructions
display_usage() {
    echo "Usage: $0 [start|stop|help]"
    echo "  start   : Start the container (default behavior if no argument is provided)"
    echo "  stop    : Stop the container"
    echo "  help    : Display this help message"
}

# Check if a command-line argument is provided
if [[ $# -eq 0 ]]; then
    # No argument provided, start the container by default
    start_container
else
    # Check the command-line argument
    case "$1" in
        start)
            start_container
            ;;
        stop)
            stop_container
            ;;
        help)
            display_usage
            ;;
        *)
            echo "Unrecognized command: $1"
            display_usage
            exit 1
            ;;
    esac
fi