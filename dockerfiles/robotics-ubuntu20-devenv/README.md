# Development environment for robotics

## Build

``` fish
docker build --tag linfenglee/robotics-ubuntu20-devenv:latest .
docker tag\
    linfenglee/robotics-ubuntu20-devenv:latest\
    linfenglee/robotics-ubuntu20-devenv:(git rev-parse --short HEAD)
```

## Run

``` fish
docker run -it --rm -p 6080:80 -v /dev/shm:/dev/shm linfenglee/robotics-ubuntu20-devenv:latest
docker run -it --rm -p 6080:80 -v /dev/shm:/dev/shm linfenglee/robotics-ubuntu20-devenv:latest bash
```

## Push

``` fish
docker push linfenglee/robotics-ubuntu20-devenv:latest
docker push linfenglee/robotics-ubuntu20-devenv:(git rev-parse --short HEAD)
```

## VNC

Map container port `80` to host port `PORT`

In side the container, run `start-novpn`, which is:
``` sh
sudo USER=joe /startup.sh
```

Then, the VNC can be accessed at `localhost:PORT` from the host

### Issue

Happened when the host is macOS: in `/startup.sh`, in the part when `USER` is not `root`:

``` sh
USER=${USER:-root}
HOME=/root
if [ "$USER" != "root" ]; then
    echo "* enable custom user: $USER"
    useradd --create-home --shell /bin/bash --user-group --groups adm,sudo $USER
    if [ -z "$PASSWORD" ]; then
        echo "  set default password to \"ubuntu\""
        PASSWORD=ubuntu
    fi
    HOME=/home/$USER
    echo "$USER:$PASSWORD" | chpasswd
    cp -r /root/{.config,.gtkrc-2.0,.asoundrc} ${HOME}
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd
fi
```
The script might get stuck at `chown -R $USER:$USER ${HOME}`, when some host dirs are mounted to container's
home dir. In this Dockerfile, the user `joe` already owns its home. Hence, we can change that line to
``` sh
chown -R $USER:$USER {.config,.gtkrc-2.0,.asoundrc}
```
so that `chown` is only applied to the newly copied files.

