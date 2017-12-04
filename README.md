# Shadow Robot Hand Config

Contains UMD-specific configuration for the Shadow Robot Hand and UR10s.

## Hardware Launch commands

To run the full system (UR10 1 and 2, the Shadow Robot Hand attached to UR10 1):

```bash
roslaunch sr_ethercat_config sr_system_umd.launch
```

To run just UR10 1 with the Shadow Robot Hand:

```bash
roslaunch sr_ethercat_config sr_system.launch
```

To run just the Shadow Robot Hand:

```bash
roslaunch sr_ethercat_config sr_rhand.launch
```

## Shadow Robot Hand Ethernet Port

If running any of the above on a machine other than the supplied tower PC, you will have to specify which ethernet port the Shadow Hand is connected to. Find the ethernet port name using `ifconfig` - look for a port with no IP configuration. The name might look like `eth0`, `enp1s0`, or `enxa0cec8126817`. To specify the name, use the `eth_port` argument, e.g.:

```bash
roslaunch sr_ethercat_config sr_system_umd.launch eth_port:=enp1s0
```

## Simulation Launch Commands

Any of the above configurations can be launched in simulation by appending the `sim` argument, e.g.:

```bash
roslaunch sr_ethercat_config sr_system_umd.launch sim:=true
```

When running in simulation, you do not have to specify the Shadow Hand ethernet port.

## Setting Up a Machine

If you want to run the system on another machine (in hardware or simulation), we strongly recommend the use of Docker, as it provides an isolated environment for you to work in, makes no changes to existing ROS workspaces on your machine, etc. To use our docker image:

1. Install and test Docker CE. You can find instructions on how to do this [here](https://docs.docker.com/engine/installation/). Make sure to add your user to the `docker` group, and log out and in before continuing.
1. Pull the docker image:

        docker pull shadowrobot/dexterous-hand:kinetic
1. If the machine has an nvidia GPU, we strongly recommend making use of it using `nvidia-docker`. If doing so:
    1. Install and test instructions are [here](https://github.com/NVIDIA/nvidia-docker). However, they instruct you to install the `nvidia-docker-2` package; don't. Instead, install the `nvidia-docker` package. You may also have to install the `nvidia-modprobe` package.
    1. 'Nvidiafy' the image:
        1. Download [the docker_nvidialize.sh script](https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/docker/utils/docker_nvidialize.sh):

                wget https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/docker/utils/docker_nvidialize.sh
        1. Run:

                ./docker_nvidialize.sh shadowrobot/dexterous-hand:kinetic
        1. You should now have a `shadowrobot/dexterous-hand:kinetic-nvidia` entry when running `docker images`. **In further steps, append `-nvidia` to the image name.**
1. Run a docker container based on the docker image:

        docker run -it -e DISPLAY -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host --privileged shadowrobot/dexterous-hand:kinetic
    Or, if using `nvidia-docker`:

        nvidia-docker run -it -e DISPLAY -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host --privileged shadowrobot/dexterous-hand:kinetic-nvidia
    In either case, there will be a short delay (up to a minute or so) while the container starts.
1. Once the container has started, pull the UMD-specific branch of `sr_config`:

        roscd sr_config
        git checkout shadowrobot_170911

    You can now run any of the above launch commands. Remember that if you are running the Shadow hand hardware, you may have to change the ethernet interface name as described above.