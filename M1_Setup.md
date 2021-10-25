# M1 Virtual Machine Setup
This document will help you setup RoboJackets Software Training on an M1 Mac using the Public Tech Preview of VMware Fusion for Apple Silicon.

## Setup
 1. Install the Public Tech Preview of VMware Fusion for Apple Silicon using this [link](https://customerconnect.vmware.com/downloads/get-download?downloadGroup=FUS-PUBTP-2021H1). 
	 - Make sure to download the ```.dmg``` file.
 2. Download the Ubuntu 20.04 desktop image for ARM using this [link](https://cdimage.ubuntu.com/focal/daily-live/current/focal-desktop-arm64.iso).
 3. Launch VMWare Fusion and click the **New** button in the Virtual Machine Library window.
 4. Select the option to **Install from disc or image** and use the Ubuntu 20.04 desktop image you downloaded in Step 3.
5. Follow the installation wizard and allow Ubuntu to install.
	- Make sure to allocate at least 4 GB of memory for your virtual machine.
6. In Ubuntu, add the ROS 2 apt repositories to your system. To do so, first authorize our GPG key with apt like this:
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```
7. And then add the repository to your sources list:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
8. Update your apt repository caches after setting up the repositories:
```bash
sudo apt update
```
9. Install ROS 2 Foxy:
```bash
sudo apt install ros-foxy-desktop
```
10. Source ROS 2 environment:
```bash
source /opt/ros/foxy/setup.bash
```
11. Create a new directory for your colcon workspace:
```bash
mkdir -p ~/training_ws/src
cd ~/training_ws/src
```
12. Clone the Software Training repo as well as the Software Training Support Library (STSL) repo.
```bash
git clone https://github.com/RoboJackets/software-training.git
git clone https://github.com/RoboJackets/stsl.git
```
	
Your folder structure should now look like this:
```
training_ws
├── src
│   ├── software-training
│   ├── stsl
```
13. Resolve package dependencies using rosdep:
```bash
cd ~/training_ws
sudo rosdep init
rosdep install -i --from-path src --rosdistro foxy -y
```
14. Install colcon:
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```
15. Now from the root of your workspace, ```training_ws```, build the packages with this command:
```bash
colcon build
```
16. Source the overlay:
```bash
source /opt/ros/foxy/setup.bash
source ~/training_ws/install/setup.bash
```
17. You Made It! :tada:
