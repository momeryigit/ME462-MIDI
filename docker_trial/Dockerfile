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

FROM ros:iron-ros-base

RUN apt-get update \
    && apt-get install -y \
    bash-completion \
    python3-argcomplete \
    nano \
    jstest-gtk \
    python3-serial \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown ${USER_UID}:${USER_GID} /home/$USERNAME/.config 

RUN apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc
ENTRYPOINT [ "/bin/bash",  "/entrypoint.sh"]
CMD ["bash"]

