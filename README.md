# Polaris Source Code

All the code related to the operation of the polaris 2019 AUVIC submarine

## Setting up your Environment

The 2 major necessary components needed to develop, build and execute the project are:
- Ubuntu 16.04 Operating System
- ROS Kinetic

Due to ROS not supporting the later Ubuntu 18.04 OS, it is currently necessary to develop on 16.04 to ensure a uniform environment.

### Ubuntu 16.04

There are several pre-requisites that a developer can fall under when setting up the necessary environment, and associated action items:
1. **Ubuntu 16.04 OS already installed**: No necessary actions needed
2. **Another Unix-like OS installed (e.g Debian, Ubuntu...) that is not Ubuntu 16.04**: Either install an Ubuntu 16.04 VM, or create an Ubuntu 16.04 docker container
3. **A Windows OS installed**: Either install an Ubuntu 16.04 VM, install Docker for Windows to run an Ubuntu 16.04 container, or install dual-boot.

**Note:** (Complete section on Windows 10 Home Virtualization issue)

### ROS Kinetic

Installation instructions are located on [ROS's website](wiki.ros.org/kinetic/Installation/Ubuntu)

**Note**: To minimize the risk of dealing with missing apt packages, ensure that `ros-kinetic-desktop-full` is installed on your environment


## Building

### Docker

1. Make sure docker is installed on your system, this is easiest on Linux and MacOS (Windows is a pain to install docker on).

2. Use the docker_make.sh script to the submarine system.

3. Use the docker scripts to ensure the images are up to date, if unsure use the last script.
    1. Use the docker_make.sh script to build the submarine.
    2. Use the docker_run.sh script to start the submarine system.
    3. Use the docker_shell.sh script to start the docker containers interactive mode (your terminal will be occupied by the docker containers terminal). This script can also be used to invoke the commands in the previous two images.

### Ubuntu 16.04 (Traditional)

1. Create a fork of the polaris repo and clone it in your ubuntu workspace `git clone https://github.com/yourusername/polaris.git`

2. Navigate to the configuration folder by `cd configurtion` and run the setup file with `./setup.sh`

3. After script has finished running, close the terminal window and open a new one. To ensure ROS has properly installed run the command `roscore`. If successful roscore will launch.

4. if successful navigate to the `ros` directory and run Run the command `catkin_make`

5. Your catkin workspace is now set up inside your local git fork.

6. Add the remote upstream with `git remote add upstream https://github.com/uvic-auvic/polaris.git`
