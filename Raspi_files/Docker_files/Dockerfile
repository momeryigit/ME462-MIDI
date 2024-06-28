# build the docker image with the following command:
# docker build -t {image name} .
# run the docker container with the following command:
# docker run -it --user docker {image name}
# some arguments:
# -v {from}:{to} : mount the volume from the host to the container
# --rm : remove the container after it is stopped
# --network=host : use the host network
#ipc=host : use the host IPC namespace
# --privileged : give the container full access to the host devices read/write
# ex: docker run -it --user dockerwros --network=host --ipc=host -v $PWD/../../dev_ws:/home/${USERNAME}/dev_ws -v /dev:/dev -v /run/udev:/run/udev -v /run/dbus/:/run/dbus --privileged my_image

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
    libopencv-dev \
    python3-opencv \
    v4l-utils \
    python3-jinja2 \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff5-dev \
    pybind11-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    python3-yaml \
    python3-ply \ 
    ninja-build \ 
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
    swig
#swig && \
##rm -rf /var/lib/apt/lists/*

# Arguments for user creation
ARG USERNAME=dockerwros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user and configure sudo access
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    mkdir /home/$USERNAME/.config && \
    chown ${USER_UID}:${USER_GID} /home/$USERNAME/.config && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG dialout ${USERNAME} && \
    usermod -aG video ${USERNAME}

# Switch to the non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

RUN sudo pip3 install meson

#Build libcamera from source
RUN sudo apt-get install -y libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev &&\
    sudo git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    sudo meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled && \
    sudo ninja -C build && \
    sudo ninja -C build install

WORKDIR /home/$USERNAME

# Build the rpicam-apps drivers from source
RUN sudo apt-get install -y libcamera-dev libjpeg-dev libtiff5-dev libpng-dev && \
    sudo apt-get install -y cmake libboost-program-options-dev libdrm-dev libexif-dev && \ 
    sudo git clone https://github.com/raspberrypi/rpicam-apps.git && \
    cd rpicam-apps && \
    sudo meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=enabled -Denable_tflite=disabled && \
    sudo meson compile -C build && \
    sudo meson install -C build && \
    sudo ldconfig -v



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