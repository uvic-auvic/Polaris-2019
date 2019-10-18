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

1. Create a fork of the polaris repo and clone it in your ubuntu workspace `git clone https://github.com/yourusername/polaris.git`.

2. If you have not already installed ROS, then navigate to the configuration folder by `cd configuration` and execute the setup script with `./setup.sh`. This can only be done with an administrator account.

3. If you have already installed ROS, please install the extra packages needed to build Polaris by navigating to the configuration folder by `cd configuration` and execute the install script with `./common-packages.sh`. This can only be done with an administrator account.

4. Navigate to polaris/ros and execute the environment variables script with `./env.sh`.

5. Close the terminal window and open a new one. To ensure ROS has properly installed, run the command `roscore`. If successful roscore will launch.

6. If successful, navigate to the `polaris/ros` directory and run the command `catkin_make`.

7. Your catkin workspace is now set up inside your local git fork.

8. Add the remote upstream with `git remote add upstream https://github.com/uvic-auvic/polaris.git`.
