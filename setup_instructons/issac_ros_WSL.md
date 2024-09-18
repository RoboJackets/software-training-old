# WSL + IsaacROS on Windows
This guide assumes a few things that you are:
- running Windows 11
- want a way to run ROS2 Humble natively without dual booting


the easier way to get ubuntu 22.04:


## 1. Chech WSL Status

run ```wsl --status```  
// wsl should already be installed in your os (windows 11) look for default version 2
* IF WSL is not installed, then follow the download [instructions] (https://learn.microsoft.com/en-us/windows/wsl/insta

find version:
```wsl -v``` equivilant to (```wsl --version```)

## 2. Install Distro
list all of your available distros of Linux:
```wsl -l -o``` equivilant to (```wsl --list --online```)

run following command to isntall Ubuntu-22.04 (alternatively you can install this on the Microsoft Store)
```wsl --install -d Ubuntu-22.04```

## 3. Bootup and Setup
Once the desired distro is installed. 
you boot by either searching for "Ubuntu 22.04" in start (or Microsoft store)
follow prompts to setup username and password 

Documentation:
[Link to Nvidia] (https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#:~:text=In%20Linux%20follow%20the%20instructions,the%20section%20above%20within%20WSL2.)


you can access it by going to the microsoft store and searching "ubuntu"
please download the Ubuntu 22.04 LTS

start wsl by typing "wsl"  in terminal
Once WSL is installed and setup, install ROS natively on WSL
[Link to Ros 2 Humble Download Instructions] (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)



## Download Docker Desktop
download docker desktop
go to settings -> go to resoruces -> wsl integration -> enable integration with additional distros
apply and restart

github clone repo for issac ros common for the dev enviorrment with 

```git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git```
clone the Isaac ros dev evniornment repo




## ISAAC ROS Conatiner

> **_NOTE_** The Docker Daemon is running via Docker Desktop on your windows computer. Your WSL isntance has a bridge/passthrough access to the Docker Daemon. the ISAAC ROS is running on WSL. 

ISAAC ROS is NVIDIA's library build on top of ROS2 and has all the capabilites of ROS2 Humble and more.
We will be using a Docker Container provided by NVIDIA that sets up the devleopment enviornment. 


then [Isaac ros dev env setup] (https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)
if on x86_64 platforms install nvidia-container-toolkit

if you have an nvidia gpu you do not need to install the driver if using wsl. 

if using docker-desktop and wsl you can not restart docker with systemctl because docekr does not exist on the wsl instance (it only has pass through) you must use the docker desktop gui 

The https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker
instuctions assume that you are running on a native linux boot. 
instead you need to:

## Configure Nvidia Container toolkit for WSL
open Docker desktop
go to settings
go to Docker Engine
insert the following code block (remember to add comma seperation after previous json element)

```
"runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
```


## Launch the Container
- launch Docker Desktop
- launch terminal
run 
```wsl```

then navigate to reposotory and launch container
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```