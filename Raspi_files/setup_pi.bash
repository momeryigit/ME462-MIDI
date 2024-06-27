#!/bin/bash

# A script to setup a Raspberry Pi for the ROMER-MIDIBOT project.
# This script will install Docker, enable camera support, build the Docker image, and run the Docker container.
# The script will also add a function to .bashrc to ensure the container is ran or executed in every new SSH session.
# chmod +x setup_pi.bash

# # Add Docker's official GPG key:
# sudo apt-get update
# sudo apt-get install -y ca-certificates curl
# sudo install -m 0755 -d /etc/apt/keyrings
# sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
# sudo chmod a+r /etc/apt/keyrings/docker.asc

# # Add the repository to Apt sources:
# echo \
#   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
#   $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
#   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
# sudo apt-get update

# # Install Docker
# sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Update and Install Docker
sudo apt-get update
sudo apt-get install -y docker.io

# Enable camera support in /boot/config.txt
echo "start_x=1" | sudo tee -a /boot/firmware/config.txt
echo "gpu_mem=128" | sudo tee -a /boot/firmware/config.txt

# Navigate to the directory containing the Dockerfile
cd ./Docker_files

image_name="midi_image"
# Build the Docker image with the name midi_image
docker build -t $image_name .


# Function to start or run the Docker container based on its status
start_or_run_container() {
    container_name="midi_container"
    container_status=$(sudo docker inspect --format="{{.State.Running}}" $container_name 2>/dev/null)

    if [ "$container_status" == "true" ]; then
        docker exec -it $container_name /bin/bash
    else
        docker run -it --user docker --name $container_name --network=host --ipc=host -v $HOME/ME462-MIDI/dev_ws:$HOME/dev_ws -v /dev:/dev -v /run/udev:/run/udev -v /run/dbus/:/run/dbus --privileged $image_name
    fi
}

# Add the container check to .bashrc to ensure it runs on every new SSH session
if ! grep -q "start_or_run_container" ~/.bashrc; then
    echo "start_or_run_container" >> ~/.bashrc
fi

# Call the function to ensure the container is running after setup
start_or_run_container