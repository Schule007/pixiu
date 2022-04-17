FROM dorowu/ubuntu-desktop-lxde-vnc:focal-lxqt

ENV DEBIAN_FRONTEND noninteractive

### Core Linux tools
RUN apt-get update
RUN apt-get install -y --no-install-recommends\
  apt-utils lsb-release sudo unzip wget ssh vim curl\
  software-properties-common gnupg2

### Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update && apt install -y --no-install-recommends ros-noetic-desktop-full

### ROS tools
RUN apt-get install -y --no-install-recommends\
    python-dev python3-dev git python3-pip\
    python3-catkin-tools python3-rosdep\
    build-essential
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### User and permissions
ARG user=joe
ARG group=joe
ARG uid=1000
ARG gid=1000
ARG home=/home/${user}
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${gid} ${group} \
    && useradd -d ${home} -u ${uid} -g ${gid} -m -s /bin/bash ${user} \
    && echo "${user} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sudoers_${user}
USER ${user}
RUN sudo usermod -a -G video ${user}
WORKDIR ${home}
ENV HOME ${home}

#################### Additional libraries ######################

### Install pinocchio

# Add robotpkg to apt
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
# Install pinocchio. Ubuntu 20 (ROS Noetic) comes with python 3.8 by default.
RUN sudo apt-get update && sudo apt install -qqy --no-install-recommends\
    robotpkg-py38-pinocchio robotpkg-py38-example-robot-data

### Franka driver 0.7.1, corresponding to firmware version 3
# Firmware version 4 can install ros-noetic-libfranka and ros-noetic-franka-ros directly
RUN sudo apt install -qqy --no-install-recommends build-essential cmake git libpoco-dev libeigen3-dev
RUN git clone --recursive https://github.com/frankaemika/libfranka ${home}/from_src/libfranka
RUN cd ${home}/from_src/libfranka &&\
    git checkout 0.7.1 &&\
    git submodule update &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release .. &&\
    MAKEFLAGS=-j$(nproc) cmake --build . &&\
    cpack -G DEB &&\
    sudo dpkg -i libfranka-0.7.1-amd64.deb
# Then build franka-ros from source using catkin
# https://github.com/frankaemika/franka_ros

### ROS control and its friends
RUN sudo apt-get update && sudo apt-get install -qqy --no-install-recommends\
    ros-noetic-ros-control ros-noetic-ros-controllers

### Pybind
RUN git clone https://github.com/pybind/pybind11.git -b stable ${home}/from_src/pybind11
RUN cd ${home}/from_src/pybind11 &&\
    cmake -S . -B build &&\
    cmake --build build -j$(nproc) &&\
    sudo cmake --install build

#################### Development tools ######################

# install emacs
RUN sudo add-apt-repository ppa:kelleyk/emacs
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends emacs27

# software
RUN sudo apt-get install -y --no-install-recommends\
    stow tmux fzf direnv python3.8-venv xsel

# python
RUN pip install --user python-lsp-server[rope,pyflakes] pytest black

# C++
RUN sudo apt-get install -y --no-install-recommends clangd-12 \
  && sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100

# install ripgrep (rg)
RUN curl -LO https://github.com/BurntSushi/ripgrep/releases/download/11.0.2/ripgrep_11.0.2_amd64.deb\
  && sudo dpkg -i ripgrep_11.0.2_amd64.deb\
  && rm ripgrep_11.0.2_amd64.deb

# colorful terminal
ENV TERM xterm-256color

# empty folders to avoid permission issue
RUN mkdir -p ${home}/dotfiles
RUN mkdir -p ${home}/workspace
RUN mkdir -p ${home}/.ssh
RUN mkdir -p ${home}/catkin_ws
RUN mkdir -p ${home}/.virtualenvs
RUN mkdir -p ${home}/.local

# modify /start.sh
RUN sudo cp -r /root/.config ${home}/.config
RUN sudo chown -R ${user}:${user} ${home}/.config
RUN sudo sed -i -e\
    's/chown -R $USER:$USER ${HOME}/chown -R $USER:$USER {.config,.gtkrc-2.0,.asoundrc}/g'\
    /startup.sh

# modify bashrc
RUN printf '\n\
export PATH=$HOME/.local/bin:/opt/openrobots/bin:$PATH\n\
export PATH=/opt/openrobots/bin:$PATH\n\
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH\n\
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH\n\
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH\n\
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH\n\
\n\
rossetup () {\n\
    if [ -z "$ROS_SETUP_DONE" ]; then\n\
        PS1="(ROS)$PS1"\n\
    fi\n\
    source /opt/ros/noetic/setup.bash\n\
    [ -f "$HOME/catkin_ws/devel/setup.bash" ] && source "$HOME/catkin_ws/devel/setup.bash"\n\
    export ROS_SETUP_DONE=1\n\
}' >> ${home}/.bashrc
############ Entrypoint ###############
ENV HOME=${home}
COPY entrypoint.bash /usr/local/bin/entrypoint
COPY start-novnc.bash /usr/local/bin/start-novnc
ENTRYPOINT ["entrypoint"]
CMD ["start-novnc"]
