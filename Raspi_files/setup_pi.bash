#!/bin/bash

# A script to setup a Raspberry Pi for the ROMER-MIDIBOT project.
# This script will install Docker, enable camera support, build the Docker image, and run the Docker container.
# The script will also add a function to .bashrc to ensure the container is ran or executed in every new SSH session.
# chmod +x setup_pi.bash
# ./setup_pi.bash

# Update and Install Docker
sudo apt-get update
sudo apt-get install -y docker.io

# File path
config_file="/boot/firmware/config.txt"

# Check and set start_x=1
if ! grep -q "^start_x=1$" "$config_file"; then
    echo "start_x=1" | sudo tee -a "$config_file" > /dev/null
    echo "Added 'start_x=1' to $config_file."
else
    echo "'start_x=1' is already set in $config_file."
fi

# Check and set gpu_mem=128
if ! grep -q "^gpu_mem=128$" "$config_file"; then
    echo "gpu_mem=128" | sudo tee -a "$config_file" > /dev/null
    echo "Added 'gpu_mem=128' to $config_file."
else
    echo "'gpu_mem=128' is already set in $config_file."
fi


# Navigate to the directory containing the Dockerfile
cd ./Docker_files

# Add the current user to the Docker group
sudo usermod -aG docker $USER


image_name="midi_image"
# Build the Docker image with the name midi_image
sudo docker build -t $image_name .


# Function to start or run the Docker container based on its status
start_or_exec_container() {
    image_name="midi_image"
    container_name="midi_container"
    container_status=$(sudo docker inspect --format="{{.State.Running}}" $container_name 2>/dev/null)

    if [ "$container_status" == "true" ]; then
        docker exec -it $container_name /bin/bash
    else
        docker start -i $container_name
    fi
}

# Define the function as a string
function_definition=$(cat <<'EOF'
start_or_exec_container() {
    image_name="midi_image"
    container_name="midi_container"
    container_status=$(sudo docker inspect --format="{{.State.Running}}" $container_name 2>/dev/null)

    if [ "$container_status" == "true" ]; then
        docker exec -it $container_name /bin/bash
    else
        docker start -i $container_name
    fi
}
EOF
)

if ! grep -q "start_or_exec_container" ~/.bashrc; then
    # If not, append the function definition to .bashrc
    echo "$function_definition" >> ~/.bashrc
    echo "Function 'start_or_exec_container' added to .bashrc."
else
    echo "Function 'start_or_exec_container' already exists in .bashrc."
fi

container_name="midi_container"
# Call the function to ensure the container is running after setup
sudo docker run -it --user dockerwros --name $container_name --network=host --ipc=host -v $HOME/ME462-MIDI:/home/dockerwros/ME462-MIDI -v /dev:/dev -v /run/udev:/run/udev -v /run/dbus/:/run/dbus --privileged $image_name

