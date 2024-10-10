# WSL + IsaacROS on Windows
This guide assumes:
- you are running Windows 11
- you want a way to run ROS2 Humble natively without dual booting

Here's an easy way to get Ubuntu 22.04 using WSL (Windows Subsystem for Linux)

> **_CODER BEWARE_** USB passthrough is finicky in WSL and might not always work with all programs 

## 1. Check WSL Status

run ```wsl --status```  
// wsl should already be installed in your os (windows 11) look for default version 2
* IF WSL is not installed, then follow the download [instructions] (https://learn.microsoft.com/en-us/windows/wsl/install)

find version:
```wsl -v``` equivilant to (```wsl --version```)

## 2. Install Distro
1. Check the list all of your available distros of Linux:

    ```wsl -l -o``` equivilant to (```wsl --list --online```)

1. Update wsl

    ```bash
    wsl --update
    ```

1. run following command to install Ubuntu-22.04 (alternatively you can install this on the Microsoft Store)

    ```wsl --install -d Ubuntu-22.04```

## 3. Bootup and Setup
Once the desired distro is installed. 
you boot by either searching for "Ubuntu 22.04" in start (or Microsoft store)
follow prompts to setup username and password 

Documentation:
[Link to Nvidia](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#:~:text=In%20Linux%20follow%20the%20instructions,the%20section%20above%20within%20WSL2.)


you can access it by going to the microsoft store and searching "ubuntu"
please download the Ubuntu 22.04 LTS

start wsl by typing "wsl"  in terminal
Once WSL is installed and setup, install ROS natively on WSL
[Link to Ros 2 Humble Download Instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)



## Download Docker Desktop
download docker desktop
go to settings -> go to resoruces -> wsl integration -> enable integration with additional distros
apply and restart


## Create your Work space
1. boot WSL (cli or microsoft store)
1. navigate home
```
cd ~
```
1. create a new working directory
```bash
mkdir -p  ~/workspaces/isaac_ros-dev/src
```
1. create a new command line variable 
```bash
echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
```
1. source bash script
```bash
source ~/.bashrc
```

github clone repo for isaac ros common for the dev enviorrment with 

1. clone the Isaac ros dev evniornment repo
``` bash
cd ${ISAAC_ROS_WS}/src/
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

## ISAAC ROS Conatiner

> **_NOTE_** The Docker Daemon is running via Docker Desktop on your windows computer. Your WSL isntance has a bridge/passthrough access to the Docker Daemon. the ISAAC ROS is running on WSL. 

ISAAC ROS is NVIDIA's library build on top of ROS2 and has all the capabilites of ROS2 Humble and more.
We will be using a Docker Container provided by NVIDIA that sets up the development enviornment. 

if you have an nvidia gpu you do not need to install the driver if using wsl. 

then [Isaac ros dev env setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)
if on x86_64 platforms install nvidia-container-toolkit


if using docker-desktop and wsl you can not restart docker with systemctl because docekr does not exist on the wsl instance (it only has pass through) you must use the docker desktop gui 

The [nvidia docker container install](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)
instuctions assume that you are running on a native linux boot. 
instead you need to:

## Configure Nvidia Container toolkit for WSL
1. open Docker desktop
2. go to settings icon in the top right
3. go to Docker Engine
4. insert the following code block (remember to add comma seperation after previous json element)

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
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

## Running Isaac Ros SLAM

# connecting realsense camera
1. set up usb passthrough to wsl
2. add device docker privildge acess in the run_dev.sh script in isaac_ros_common 

cd ${ISAAC_ROS_WS} && \
colcon build --symlink-install --packages-up-to-regex realsense*


??? in development

## USB passthrough 
[usb passthrough offical microsoft instructions](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

launch as admin

1. download `.msi` file
[usbip download](https://github.com/dorssel/usbipd-win/releases)

2. list all usb devices
    ```bash 
    usbpid list
    ```
3. change usb to being shared:
    ```bash
    usbipd bind --busid <busid>
    ```
4. check if usb is shared
    ```
    usbipd list
    ```
5. launch wsl 

6. attach usb to wsl
    ```
    usbipd attach --wsl --busid <busid>
    ```

7. check if the usb is visible in ubuntu wsl
    ```
    lsusb
    ```
8. Check to see where new usb device is conencted
> in my case where I attached a d455f camera i got a ```/dev/usb/hiddev0``` directory
```
ls /dev/
```

## USB Disconnect
1. Disconnect physically or run command:
    ```
    usbipd detach --busid <busid>
    ```