#!/usr/bin/env bash

docker run -d\
       -p 6080:80\
       -p 8800:8888\
       -v /dev/shm:/dev/shm\
       -v /Users/linfeng/workspace:/home/joe/workspace\
       -v renv-dotfiles:/home/joe/dotfiles\
       -v renv-catkin-ws:/home/joe/catkin_ws\
       -v home-ssh:/home/joe/.ssh\
       -e DISPLAY=:1\
       -e RESOLUTION=1920x1080\
       --name renv\
       -w="/home/joe"\
       --init\
       linfenglee/robotics-ubuntu20-devenv:latest sleep infinity
