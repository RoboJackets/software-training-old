# Setup Instructions

The Robot Operating System (ROS) runs best in Ubuntu Linux. Since most students aren't running Linux on their personal computers, we provide instructions below for setting up a virtual machine that will let you run emulate running Linux on your computer.

Virtualization can get complicated. There are many options for VM technology and graphical front ends. Some work better than others in certain scenarios, and your host operating system places some constraints on which ones you can use. For each type of machine listed below, we've provided multiple VM options. You should pick one to use for software training.

We've marked the solution we recommend for each scenario. These are the options that we feel are most reliable and easiest for us to help you with should you have any problems.

## ROS2 and IsaacROS
The teams with access to a Jetson platform are using ISAAC ROS built and maintained by NVIDIA. ISAAC ROS is built on ROS 2 Humble and has all the capabilites of ROS 2 Humble and can nominally be used as such. ISAAC ROS comes with a few libraries that can prove to be useful but can only be run optimized on Nvidia hardware (use the isaac_ros development container) [instructions] (isaac_ros_WSL.md)

## Recomeded Method For ALL Operating Systems:
> ** _CODER BEWARE_ ** usb/peripherals devices may not run properly on docker due to passthrough. 
- [Docker](docker.md)


## Legacy Instructions
Please note that docker is the only LTS setup as of right now regardless of OS. Note: we have had issues with docker and VNC on some apple silicon Macs, the issue has not be reliabley reproduceable so if you run into any issues please create a ticket and try other methods of running ubuntu. 

### For Windows Computers
If you have an NVIDIA GPU we recomend using WSL for best use of the CUDA acceration
- [Windows Subsystem for Linux](wsl.md) 
- [VirtualBox](virtualbox.md)

### For Intel-based Apple Computers

- [VirtualBox](virtualbox.md)

### For Apple Silicon Computers (M1 / M2)

- [Docker](docker.md)
- [VMware Fusion](vmware_fusion.md)

### For Linux Computers

- [Linux Setup for amd64](bare_metal_ubuntu_amd64.md)
- [Linux Setup for arm64](bare_metal_ubuntu_arm64.md)
- [VirtualBox](virtualbox.md)
