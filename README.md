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
        2. Connecting to `eduroam`:
            1. Set "Wi-Fi security" to `WPA & WPA2 Enterprise`.
            2. Set "Authentication" to `Protected EAP (PEAP)`.
            3. Leave "Anonymous identity" blank.
            4. Set "Domain" to `deakin.edu.au`.
            5. Check the "No CA certificate is required" box.
            6. Set "PEAP version" to `Automatic`.
            7. Set "Inner authentication" to `MSCHAPv2`.
            8. Set "Username" to your Deakin Uni username (e.g. `ashau`).
            9. Set "Password" to your Deakin Uni password.
    5. Test the internet connection by opening Firefox and attempting to
        load a random page.
    6. Install Network Tools + Ping Debugger in Terminal.
        ``` bash
        $ sudo apt install net-tools iputils-ping
        ```
2. Install ROS Melodic.
    1. Setup the computer to accept software from packages.ros.org.
        ``` bash
        $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        ```
    2. Install Curl.
        ``` bash
        $ sudo apt install curl
        ```
    3. Setup ROS Keys.
        ``` bash
        $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        ```
    4. Update Debian Packages.
        ``` bash
        $ sudo apt update
        ```
    5. Install ROS Melodic Desktop Full.
        ``` bash
        $ sudo apt install ros-melodic-desktop-full
        ```
    6. Source ROS Melodic.
        ``` bash
        $ source /opt/ros/melodic/setup.sh
        ```
    7. Add aliases to simplify sourcing ROS Melodic in the future.
        ``` bash
        $ echo "alias source_ros='source /opt/ros/melodic/setup.sh'" >> ~/.bashrc
        $ echo "alias source_ws='source devel/setup.sh'" >> ~/.bashrc
        ```
    8. Install rosdep (used for installing dependencies).
        ``` bash
        $ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
        ```
    9. Initialize rosdep.
        ``` bash
        $ sudo rosdep init
        ```
    10. Update rosdep for ROS Melodic.
        ``` bash
        $ rosdep update --rosdistro melodic
        ```
    11. Install ROS Melodic Controllers.
        ``` bash
        $ sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
        ```
    12. Update, Upgrade, and Remove Debian Packages.
        ``` bash
        $ sudo apt update # updates the lists of packages available
        $ sudo apt upgrade # upgrades / installs new packages
        $ sudo apt autoremove # removes non-required packages
        ```
3. Restart the Ubuntu VM.
4. Create a ROS workspace for the GO1 robot.
    1. Open a new terminal.
    2. Create a new ROS workspace.
        ``` bash
        $ mkdir -p ~/ros_ws/src
        $ cd ~/ros_ws/src
        ```
    3. Source ROS.
        ``` bash
        $ source_ros # already defined this alias previously
        ```
    4. Install the Unitree GO1 GIT repositories.
        ``` bash
        $ git clone https://github.com/unitreerobotics/unitree_ros -b master
        $ git clone https://github.com/unitreerobotics/unitree_ros_to_real -b master
        $ git clone https://github.com/unitreerobotics/unitree_legged_sdk -b go1
        ```
    5. Install project dependencies.
        ``` bash
        $ cd .. # after this, you should be in the `ros_ws` directory
        $ rosdep install --from-paths src -y --ignore-src --rosdistro melodic
        ```
    6. Build the project.
        ``` bash
        $ catkin_make
        ```
5. Connect Ubuntu VM to GO1 Robot.
    1. Turn on the robot. This process is shown in this [video](https://www.youtube.com/watch?v=VbabuAhol0E).
    2. Connect via Ethernet.
        1. If you plug in the Ethernet cable, but are unable to connect, try
            the following to fix the issue:
            1. Set Static Port
                1. Get Ethernet Port Name:
                    ``` bash
                    $ ifconfig
                    enp0s25: ... # en* means ethernet - this is the one we want
                        ...
                    lo: ...
                        ...
                    wlp3so: ...
                        ...
                    ```
                2. Set static connection:
                    ``` bash
                    $ sudo ifconfig enp0s25 down # replace `enp0s25` if required
                    $ sudo ifconfig enp0s25 192.168.123.162/24
                    $ sudo ifconfig enp0s25 up
                    ```
                3. Test Connection:
                    ``` bash
                    $ ping -c 3 192.168.123.163 # -c 3 means ping 3 times
                    ```
    <!-- 2. Connect to the Unitree GO1 robot wireless access point. -->
    3. Run a example.
        1. Open up 2 terminals (`A` and `B`).
            1. In `A`, input the following commands:
                ``` bash
                $ cd ~/ros_ws
                $ source_ws
                $ roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
                ```
            2. In `B`, input the following commands (after having run all of
                the `A` commands):
                ``` bash
                $ cd ~/ros_ws
                $ source_ws
                $ rosrun unitree_legged_real ros_example_walk
                ```