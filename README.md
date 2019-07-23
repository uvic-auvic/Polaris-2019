# Polaris Source Code

All the code related to the operation of the polaris 2019 AUVIC submarine

## Setting up your Environment

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
