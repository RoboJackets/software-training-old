* RoboJackets Docker Image Setup Guide

If you would prefer to use Docker (or your computer does not support VirtualBox) follow this guide.

1. Download Docker: https://docs.docker.com/get-docker/
2. In your terminal run: `docker pull thecharlesjenkins/robojackets-software-training`
3. Run `docker run  --rm -p 6080:80 -p 6081:443 --mount source=rj-training-home,destination=/home/padowan --mount source=rj-training-usr,destination=/usr -e USER=padowan -e PASSWORD=robojackets --name training thecharlesjenkins/robojackets-software-training -h training`.
4. Navigate to `localhost:6080` in your browser and wait (it may take a while).
5. Start training! You will also have access to the repos for robonav, roboracing, and robocup.
6. Anytime password is needed fill in with `robojackets` or whatever you put as the argument to the environment variable in step 3.

If the above does not work on you computer, you can replace steps 2 and three with:
2. `docker pull thecharlesjenkins/robojackets-software-training:m1`
3. Run `docker run  --rm -p 5900:5900 --mount source=rj-training-home,destination=/home/padowan --mount source=rj-training-usr,destination=/usr -e USER=padowan -e PASSWORD=robojackets --name training thecharlesjenkins/robojackets-software-training:m1 -h training`
4. Use a VNC viewer to navigate to localhost:5900. On Safari you can do `vnc://localhost:5900`