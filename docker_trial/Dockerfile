# build the docker image with the following command:
# docker build -t {image name} .
# run the docker container with the following command:
# docker run -it --user ros {image name}
# some arguments:
# -v {from}:{to} : mount the volume from the host to the container
# --rm : remove the container after it is stopped
# --network=host : use the host network
#ipc=host : use the host IPC namespace
# --privileged : give the container full access to the host devices read/write
# ex: docker run -it --user ros --network=host --ipc=host -v $PWD/../dev_ws:/dev_ws -v /dev:/dev --privileged my_image

# Use the official ROS iron base image
FROM ros:iron-ros-base

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
    bash-completion \
    python3-argcomplete \
    nano \
    jstest-gtk \
    python3-serial \
    iputils-ping \
    python3-smbus \
    sudo \
    cmake \
    pkg-config \
    python3-pip \
    swig && \
    rm -rf /var/lib/apt/lists/*

# Arguments for user creation
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user and configure sudo access
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    mkdir /home/$USERNAME/.config && \
    chown ${USER_UID}:${USER_GID} /home/$USERNAME/.config && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG dialout ${USERNAME}

# Switch to the non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Clone, build, and install YDLidar SDK
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    mkdir -p YDLidar-SDK/build && \
    cd YDLidar-SDK/build && \
    cmake .. && \
    make && \
    sudo make install && \
    cd /home/$USERNAME/YDLidar-SDK && \
    pip install .

# Switch back to root user to change permissions of entrypoint.sh
USER root

# Copy the entrypoint script and .bashrc
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Change ownership and permissions of the entrypoint script
RUN chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc && chmod +x /entrypoint.sh

# Switch back to the non-root user
USER $USERNAME

# Set the entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]