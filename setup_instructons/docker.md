# Fall 2024 Enviornment SETUP

## Docker Installation

Doing a Docker installation is a great method for setting up the repository on any operating system
that isn't Ubuntu 22.04. It is a faster and more lightweight alternative to a traditional Virtual Machine.
In addition, you can still run GUI applications like Gazebo using the NoVNC desktop environment.

## 1. Install Docker

[Windows Instructions](https://docs.docker.com/desktop/windows/install/)

[Mac Instructions](https://docs.docker.com/desktop/mac/install/)

[Ubuntu Instructions](https://docs.docker.com/engine/install/ubuntu/)

### NOTE
* If you are on Linux, add yourself to the `docker` group. Being a member of the `docker` group allows you to run `docker` without `sudo`.
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```

After you complete the installation, **restart your computer**!

To check that everything installed OK, you should be able to open the command line and type:
```bash
docker
```

## 2. Install VS Code (Highly Recommended)

VS Code is the text editor of choice for most veteran RoboJackets members, thanks to its robust library of helpful extensions.

[Download VS Code here](https://code.visualstudio.com/Download)

### 2a. Install VS Code Extensions

Search for and install the following extensions in VS Code

* Docker
* ROS
* C/C++
* CMake

## 3. Install Git

[Install Git using the instructions here](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

## 4. Create directory to mount container

The docker container is essentially a self-contained instance of Ubuntu 22.04, with access to any files in the directory you mount it in. We do this so the container can see the repositories you clone locally, that way both local development and containerized testing can be done seamlessly. 

**You can place this directory wherever you want**, I recommend `/home` for Mac/Linux and `C:\Users\[Username]\` for Windows

```bash
mkdir rj_training_container
```

## 5. [Download our installation script](setup.sh)

Our installation script will:
- Make sure you have all necessary software
- Clone all necessary repos
- Pull the Docker image
- Automatically setup your desktop NoVNC environment

## 6. Move the script into your `rj_training_container` directory

## 7. Run the script

**If on Windows, you will need to use Git Bash to run the following commands!**

Run this command **only if you are on Mac/Linux**
```bash
chmod +x setup.sh
```

```bash
./setup.sh
```

## 8. Access your new container

For beginners to Docker:
- Go to `localhost:6060` in your web browser of choice

Recommended way:
- Open up VS Code
- Click on the whale Docker icon on your left
- Right click the currently running container (Green arrow next to name)
- Select open in browser

![Picture of where to head in VS Code](../pictures/docker_tab.png)

## 9. Head to your mounted directory

Open terminator on the desktop (this is the recommended terminal for commands in your container)

right click on terminal -> profiles -> profile preferences -> scrolling -> set to a pretty high number

![You can find terminator at the top right](../pictures/terminator_location.png)

In terminator, run
```bash
cd rj_training_container
```

## 10. Get necessary packages

First, it's always a good idea to check for updates. Nothing will happen if you just created the image. However, if you decide to re-create the container a while after you made the initial image, you will need to update those packages.

```bash
sudo apt update
sudo apt upgrade
```





## Software Training Support Library Download
This section is now updated via bash set up script
```bash
git clone https://github.com/RoboJackets/stsl.git
```


## isntall ROS Dependencies
```bash
cd /training_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```


## Colcon Build
go back to training_ws
```bash
cd ..
```


```bash
colcon build
```


```bash
source install/setup.bash
```




## DEBUG COMMANDS

get the apt package for a given ros package

```
rosdep resolve <package>
```
search for things
```bash
apt-cache search
```
shows you info

```bash
apt-cache policy
```
see what src packages are curretly available to us
```bash
colcon list
```
