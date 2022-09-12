# VirtualBox

**TODO** These instructions still need to be reviewed / updated.

This document will help you get started using the RoboJackets Software Virtualbox Image

1. Install VirtualBox from the [VirtualBox downloads page](https://www.virtualbox.org/wiki/Downloads).

1. [Windows Only] If you are on Windows, ensure you have Virtualization turned on in your BIOS.

   Follow [this guide](http://www.howtogeek.com/213795/how-to-enable-intel-vt-x-in-your-computers-bios-or-uefi-firmware/).

   While this is not 100% necessary, it will make your VM _much_ faster.

   You may also need to turn off Hyper-V.

1. Get a Copy of the built Image

   First start by downloading the [prebuild image](https://mirror.robojackets.org/software_training_fall_2021.ova).

1. Import Image into VirtualBox

   1. Go to `File->Import Appliance`

     ![](https://i.imgur.com/MbxOAH7.png)

   1. Select the `.ova` file you downloaded earlier

     ![](https://i.imgur.com/LbBx78G.png)

   1. Check the Amount of Memory and CPU's

     Increase the Memory/CPU to your computer's specs. Don't allocate too much memory/cpus or it will cause severe performance issues.
     (A good rule of thumb is half of your overall memory if you have at least 4 GB and half of your total CPU's)

     ![](https://i.imgur.com/4O0l8hN.png)

   1. Hit `Import`!
     
1. Boot your new VM

   Double Click the Entry, or Right Click -> Start -> Normal Start

1. Credentials

   | User        | Password    |
   | ----------- | ----------- |
   | robojackets | robojackets |

   The sudo password is also `robojackets`.

1. You Made It! :tada:

   If you are having issues see the troubleshooting section below
   
## Troubleshooting Steps

1. If you get `Unable to import OVA with error NS_ERROR_INVALID_ARG` it could be caused by not having enough hard drive space to unpack the image.

1. If you get a `Kernel Driver not installed` error try the instructions here: https://www.howtogeek.com/658047/how-to-fix-virtualboxs-%E2%80%9Ckernel-driver-not-installed-rc-1908-error/

1. Check the Virtual Machine Settings
   1. If you are getting a black screen when attempting to run gazebo or other 3D applications try disabling 3D Acceleration by right clicking the virtualbox entry and hitting `Settings`. Then, go to the `Display` tab and uncheck the `Enable 3D Acceleration` box. 
   1. If you are having other graphical issues check how much VRAM is allocated by right clicking the virtualbox entry and hitting `Settings`. Then, go to the `Display` tab and confirm that `VRAM` is set to 128 MB and the `Enable 3D Acceleration` checkbox is checked. 
      
   1. Check to see if too much memory/cpu's have been allocated to the virtual machine by right clicking the virtualbox entry and hitting `Settings`. Then, go to the `System` tab and check that the `Memory` slider on the `Motherboard` tab is set to around half of your total system RAM. Finally, go to the `Processor` tab and check that the `Processor(s)` slider is set to between 1 and half of your total system CPU cores and that the `Exectuion Cap` is set to 100%.
