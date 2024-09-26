# Base image(Isaac Sim)
ARG ISAACSIM_VERSION_ARG=4.2.0
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION_ARG} AS base
ENV ISAACSIM_VERSION=${ISAACSIM_VERSION_ARG}

# Set default RUN shell to bash
SHELL ["/bin/bash", "-c"]

# Envs
ENV ACCEPT_EULA=Y
ENV ISAACSIM_ROOT_PATH=/isaac-sim
ENV ISAACLAB_PATH=/workspace/isaaclab
ENV SIMTOREAL_PATH=/workspace/sim_to_real
ENV DOCKER_USER_HOME=/root
ENV LANG=C.UTF-8
# in case get interupted
ENV DEBIAN_FRONTEND=noninteractive  

USER root

# Install dependencies and remove cache
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libglib2.0-0 \
    ncurses-term \
    python3 \
    python3-pip \
    wget \
    unzip && \
    apt -y autoremove && apt clean autoclean && \
    rm -rf /var/lib/apt/lists/*

    # sim_to_real repo
ARG SIMTOREAL_REPO_URL=https://github.com/XMebius/isaaclab_sim_to_real.git
RUN git clone ${SIMTOREAL_REPO_URL} ${SIMTOREAL_PATH}
RUN cd ${SIMTOREAL_PATH} && \
    pip install -e .

# LCM
ARG LCM_ROOT_PATH=/workspace/
RUN cd ${LCM_ROOT_PATH} && \
    wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip && \
    unzip lcm-1.5.0.zip && \
    rm lcm-1.5.0.zip

# Build and install LCM core
RUN mkdir ${LCM_ROOT_PATH}/lcm-1.5.0/build && \
    cd ${LCM_ROOT_PATH}/lcm-1.5.0/build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig

# Install the Python bindings
RUN cd ${LCM_ROOT_PATH}/lcm-1.5.0/lcm-python && \
python3 setup.py install

# Clean up the working directory
RUN rm -rf ${LCM_ROOT_PATH}/lcm-1.5.0

# clone Isaac Lab repo
ARG ISAACLAB_REPO_URL=https://github.com/isaac-sim/IsaacLab.git
ARG ISAACLAB_REPO_TAG=v1.2.0
RUN git clone --branch ${ISAACLAB_REPO_TAG} ${ISAACLAB_REPO_URL} ${ISAACLAB_PATH}

RUN ln -sf ${ISAACSIM_ROOT_PATH} ${ISAACLAB_PATH}/_isaac_sim

# Install apt dependencies for extensions 
RUN --mount=type=cache,target=/var/cache/apt \
    ${ISAACLAB_PATH}/isaaclab.sh -p ${ISAACLAB_PATH}/tools/install_deps.py apt ${ISAACLAB_PATH}/source/extensions && \
    apt -y autoremove && apt clean autoclean && \
    rm -rf /var/lib/apt/lists/*

# create the directories that will binded
RUN mkdir -p ${ISAACSIM_ROOT_PATH}/kit/cache && \
    mkdir -p ${DOCKER_USER_HOME}/.cache/ov && \
    mkdir -p ${DOCKER_USER_HOME}/.cache/pip && \
    mkdir -p ${DOCKER_USER_HOME}/.cache/nvidia/GLCache &&  \
    mkdir -p ${DOCKER_USER_HOME}/.nv/ComputeCache && \
    mkdir -p ${DOCKER_USER_HOME}/.nvidia-omniverse/logs && \
    mkdir -p ${DOCKER_USER_HOME}/.local/share/ov/data && \
    mkdir -p ${DOCKER_USER_HOME}/Documents

# create NVIDIA binary placeholders
RUN touch /bin/nvidia-smi && \
    touch /bin/nvidia-debugdump && \
    touch /bin/nvidia-persistenced && \
    touch /bin/nvidia-cuda-mps-control && \
    touch /bin/nvidia-cuda-mps-server && \
    touch /etc/localtime && \
    mkdir -p /var/run/nvidia-persistenced && \
    touch /var/run/nvidia-persistenced/socket

# Isaac Lab dependencies
RUN --mount=type=cache,target=${DOCKER_USER_HOME}/.cache/pip \
    ${ISAACLAB_PATH}/isaaclab.sh --install

# aliase
RUN echo "export ISAACLAB_PATH=${ISAACLAB_PATH}" >> ${HOME}/.bashrc && \
    echo "alias isaaclab=${ISAACLAB_PATH}/isaaclab.sh" >> ${HOME}/.bashrc && \
    echo "alias python=${ISAACLAB_PATH}/_isaac_sim/python.sh" >> ${HOME}/.bashrc && \
    echo "alias python3=${ISAACLAB_PATH}/_isaac_sim/python.sh" >> ${HOME}/.bashrc && \
    echo "alias pip='${ISAACLAB_PATH}/_isaac_sim/python.sh -m pip'" >> ${HOME}/.bashrc && \
    echo "alias pip3='${ISAACLAB_PATH}/_isaac_sim/python.sh -m pip'" >> ${HOME}/.bashrc && \
    echo "alias tensorboard='${ISAACLAB_PATH}/_isaac_sim/python.sh ${ISAACLAB_PATH}/_isaac_sim/tensorboard'" >> ${HOME}/.bashrc && \
    echo "export TZ=$(date +%Z)" >> ${HOME}/.bashrc

WORKDIR ${SIMTOREAL_PATH}