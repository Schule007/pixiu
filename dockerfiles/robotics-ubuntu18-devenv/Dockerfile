FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt

ENV DEBIAN_FRONTEND noninteractive

### Core Linux tools
RUN apt-get update
RUN apt-get install -y --no-install-recommends\
  apt-utils lsb-release sudo unzip wget ssh vim curl\
  software-properties-common gnupg2

### Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update && apt install -y --no-install-recommends ros-melodic-desktop-full

### ROS tools
RUN apt-get install -y --no-install-recommends\
    python-dev python-dev git python-pip\
    python-catkin-tools python-rosdep\
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

### ROS control and its friends
RUN sudo apt-get update && sudo apt-get install -qqy --no-install-recommends\
    ros-melodic-ros-control ros-melodic-ros-controllers

### ROS
RUN sudo apt-get install ros-melodic-ros-numpy

### iai-kinect2

# libfreenect2
RUN git clone --recursive https://github.com/OpenKinect/libfreenect2.git ${home}/from_src/libfreenect2
RUN cd ${home}/from_src/libfreenect2\
    && sudo apt-get install -y --no-install-recommends\
    build-essential cmake pkg-config\
    libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev\
    && mkdir build && cd build\
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr\
    && make -j$(nproc) && sudo make install\
    && sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

### Kinova
# ros_kortex_vision
RUN sudo apt-get install gstreamer1.0-tools gstreamer1.0-libav\
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev\
    libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good\
    gstreamer1.0-plugins-base
RUN sudo apt-get install ros-melodic-rgbd-launch

#################### Development tools ######################

# install emacs
RUN sudo add-apt-repository ppa:kelleyk/emacs
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends emacs27

# software
RUN sudo apt-get install -y --no-install-recommends\
    stow tmux xsel
RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && ~/.fzf/install

# C++
# RUN sudo apt-get install -y --no-install-recommends clangd-12 \
#   && sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100

# conan
RUN sudo apt-get install -y --no-install-recommends python3 python3-pip python3-setuptools \
    && sudo python3 -m pip install conan
# RUN curl -LO https://github.com/conan-io/conan/releases/latest/download/conan-ubuntu-64.deb\
#     && sudo dpkg -i conan-ubuntu-64.deb\
#     && rm conan-ubuntu-64.deb
RUN conan config set general.revisions_enabled=1\
    && conan profile new default --detect > /dev/null\
    && conan profile update settings.compiler.libcxx=libstdc++11 default

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
rossetup () {\n\
    if [ -z "$ROS_SETUP_DONE" ]; then\n\
        PS1="(ROS)$PS1"\n\
    fi\n\
    source /opt/ros/melodic/setup.bash\n\
    [ -f "$HOME/catkin_ws/devel/setup.bash" ] && source "$HOME/catkin_ws/devel/setup.bash"\n\
    export ROS_SETUP_DONE=1\n\
}' >> ${home}/.bashrc
############ Entrypoint ###############
ENV HOME=${home}
COPY entrypoint.bash /usr/local/bin/entrypoint
COPY start-novnc.bash /usr/local/bin/start-novnc
ENTRYPOINT ["entrypoint"]
CMD ["start-novnc"]
