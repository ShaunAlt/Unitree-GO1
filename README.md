# Unitree-GO1
Unitree GO1 Robot Connection + Control Project

## Table of Contents
- [Contributors](#contributors)
- [Connecting to GO1 Robot](#connecting-to-go1-robot)

## Contributors
This project was created by Shaun Altmann (shaun.altmann@deakin.edu.au).

## Connecting to GO1 Robot
The following steps will allow you to connect to the Unitree GO1 Robot:
1. Create new Ubuntu 18.04 VM.
    1. Install Ubuntu 18.04 Desktop Image from this
        [link](https://releases.ubuntu.com/18.04/).
    2. Create the Ubuntu VM with the 18.04 AMD image (ARM has not been tested).
    3. Run the Ubuntu VM.
    4. Connect the Ubuntu VM to an internet connection.
        1. If at Deakin Uni, you can use the `Guest_WiFi_Deakin` network.
    5. Test the internet connection by opening Firefox and attempting to
        load a random page.
2. Install ROS Melodic.
    1. Setup the computer to accept software from packages.ros.org.
        ``` bash
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        ```
    2. Install Curl.
        ``` bash
        sudo apt install curl
        ```
    3. Setup ROS Keys.
        ``` bash
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        ```
    4. Update Debian Packages.
        ``` bash
        sudo apt update
        ```
    5. Install ROS Melodic Desktop Full.
        ``` bash
        sudo apt install ros-melodic-desktop-full
        ```
    6. Source ROS Melodic.
        ``` bash
        source /opt/ros/melodic/setup.sh
        ```
    7. Add an alias to simplify sourcing ROS Melodic in the future.
        ``` bash
        echo "alias source_ros='source /opt/ros/melodic/setup.sh'" >> ~/.bashrc
        ```
    8. Install rosdep (used for installing dependencies)
        ``` bash
        sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
        ```
    9. Initialize rosdep.
        ``` bash
        sudo rosdep init
        ```
    10. Update rosdep for ROS Melodic.
        ``` bash
        rosdep update --rosdistro melodic
        ```
