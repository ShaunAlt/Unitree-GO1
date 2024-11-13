# Unitree-GO1
Unitree GO1 Robot Connection + Control Project

## Table of Contents
- [Contributors](#contributors)
- [Creating an Ubuntu VM](#creating-an-ubuntu-vm)
    - [Ubuntu 18.04 (AMD)](#ubuntu-1804-amd)
    - [Ubuntu 20.04 (ARM)](#ubuntu-2004-arm)
    - [Ubuntu 22.04 (ARM)](#ubuntu-2204-arm)
- [Connecting to Eduroam](#connecting-to-eduroam)
- [Installing ROS](#installing-ros)
    - [ROS1 Melodic](#ros1-melodic)
    - [ROS1 Noetic](#ros1-noetic)
    - [ROS2 Humble](#ros2-humble)
    - [ROS2 Iron](#ros2-iron)
- [Connecting to GO1 Robot](#connecting-to-go1-robot)
    - [Turning on the Robot](#turning-on-the-robot)
    - [Ethernet Connection](#ethernet-connection)
    - [WiFi Connection](#wifi-connection)

## Contributors
This project was created by Shaun Altmann (shaun.altmann@deakin.edu.au).

## Creating an Ubuntu VM
### Ubuntu 18.04 (AMD)
1. Install the [Ubuntu 18.04 AMD64 Desktop Image](https://releases.ubuntu.com/18.04/ubuntu-18.04.6-desktop-amd64.iso),
    which can be found [here](https://releases.ubuntu.com/18.04/).
2. Create the Ubuntu VM using the 18.04 image.
3. Run the Ubuntu VM.
4. Open the Terminal and install Net Tools + IP Config.
    ``` bash
    $ sudo apt install net-tools iputils-ping
    ```

### Ubuntu 20.04 (ARM)
1. Install the [Ubuntu 20.04 64-bit ARM Server Image](https://cdimage.ubuntu.com/releases/focal/release/ubuntu-20.04.5-live-server-arm64.iso),
    which can be found [here](https://cdimage.ubuntu.com/releases/focal/release/).
2. Create the Ubuntu VM using the 20.04 image.
3. Run the Ubuntu VM.
4. Install Ubuntu Desktop on the Server.
    ``` bash
    $ sudo apt install ubuntu-desktop
    $ sudo reboot
    ```
5. Open the Terminal and install Net Tools + IP Config.
    ``` bash
    $ sudo apt install net-tools iputils-ping
    ```

### Ubuntu 22.04 (ARM)
1. Install the [Ubuntu 22.04 64-bit ARM Server Image](https://cdimage.ubuntu.com/releases/jammy/release/ubuntu-22.04.5-live-server-arm64.iso),
    which can be found [here](https://cdimage.ubuntu.com/releases/jammy/release/).
2. Create the Ubuntu VM using the 22.04 image.
3. Run the Ubuntu VM.
4. Install Ubuntu Desktop on the Server.
    ``` bash
    $ sudo apt install ubuntu-desktop
    $ sudo reboot
    ```
5. Open the Terminal and install Net Tools + IP Config.
    ``` bash
    $ sudo apt install net-tools iputils-ping
    ```

## Connecting to Eduroam
Use the following settings to connect to eduroam.
| Setting | Value |
| :--- | ---: |
| Wi-Fi security | WPA & WPA2 Enterprise |
| Authentication | Protected EAP (PEAP) |
| Anonymous identity | _\*\*leave blank\*\*_ |
| Domain | deakin.edu.au |
| No CA certificate is required | <ul><li>- [x] </ul> |
| PEAP version | Automatic |
| Inner authentication | MSCHAPv2 |
| Username | _\*\*your deakin uni login username\*\*_ |
| Password | _\*\*your deakin uni login password\*\*_ |

## Installing ROS
| ROS Version | Ubuntu Requirement |
| :--- | ---: |
| ROS1 Melodic | 18.04 (Bionic) |
| ROS1 Noetic | 20.04 (Focal) |
| ROS2 Humble | 22.04 (Jammy) |
| ROS2 Iron | 22.04 (Jammy) |

### ROS1 Melodic
The steps for this process can be found [here](https://wiki.ros.org/melodic/Installation/Ubuntu).
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
11. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
12. Upgrade Debian Packages.
    ``` bash
    $ sudo apt upgrade
    ```
13. Remove Non-Required Debian Packages.
    ``` bash
    $ sudo apt autoremove
    ```
14. Reset your VM.
    ``` bash
    $ sudo reboot
    ```

### ROS1 Noetic
The steps for this process can be found [here](https://wiki.ros.org/noetic/Installation/Ubuntu).
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
5. Install ROS Noetic Desktop Full.
    ``` bash
    $ sudo apt install ros-noetic-desktop-full
    ```
6. Source ROS Noetic.
    ``` bash
    $ source /opt/ros/noetic/setup.sh
    ```
7. Add aliases to simplify sourcing ROS Noetic in the future.
    ``` bash
    $ echo "alias source_ros='source /opt/ros/noetic/setup.sh'" >> ~/.bashrc
    $ echo "alias source_ws='source devel/setup.sh'" >> ~/.bashrc
    ```
8. Install rosdep (used for installing dependencies).
    ``` bash
    $ sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```
9. Initialize rosdep.
    ``` bash
    $ sudo rosdep init
    ```
10. Update rosdep for ROS Noetic.
    ``` bash
    $ rosdep update --rosdistro noetic
    ```
11. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
12. Upgrade Debian Packages.
    ``` bash
    $ sudo apt upgrade
    ```
13. Remove Non-Required Debian Packages.
    ``` bash
    $ sudo apt autoremove
    ```
14. Reset your VM.
    ``` bash
    $ sudo reboot
    ```

### ROS2 Humble
The steps for this process can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
1. Ensure that the [Ubuntu Universe Repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.
    ``` bash
    $ sudo apt install software-properties-common
    $ sudo add-apt-repository universe
    ```
2. Install Curl.
    ``` bash
    $ sudo apt install curl -y
    ```
3. Setup ROS GPG Key.
    ``` bash
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
4. Add the repository to your sources list.
    ``` bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
5. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
6. Install ROS Development Tools.
    ``` bash
    $ sudo apt install ros-dev-tools
    ```
7. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
8. Upgrade Debian Packages.
    ``` bash
    $ sudo apt upgrade
    ```
9. Install ROS Humble Desktop Full.
    ``` bash
    $ sudo apt install ros-humble-desktop
    ```
10. Add aliases to simplify sourcing ROS2 Humble in the future.
    ``` bash
    $ echo "alias source_ros2='source /opt/ros/humble/setup.sh'" >> ~/.bashrc
    $ echo "alias source_ws2='source install/setup.sh'" >> ~/.bashrc
    ```
11. Install rosdep (used for installing dependencies).
    ``` bash
    $ sudo apt install python3-rosdep
    ```
12. Initialize rosdep.
    ``` bash
    $ sudo rosdep init
    ```
13. Update rosdep for ROS Humble.
    ``` bash
    $ rosdep update --rosdistro humble
    ```
14. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
15. Upgrade Debian Packages.
    ``` bash
    $ sudo apt upgrade
    ```
16. Remove Non-Required Debian Packages.
    ``` bash
    $ sudo apt autoremove
    ```
17. Reset your VM.
    ``` bash
    $ sudo reboot
    ```

### ROS2 Iron
The steps for this process can be found [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html).
1. Ensure that the [Ubuntu Universe Repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.
    ``` bash
    $ sudo apt install software-properties-common
    $ sudo add-apt-repository universe
    ```
2. Install Curl.
    ``` bash
    $ sudo apt install curl -y
    ```
3. Setup ROS GPG Key.
    ``` bash
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
4. Add the repository to your sources list.
    ``` bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
5. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
6. Install ROS Development Tools.
    ``` bash
    $ sudo apt install ros-dev-tools
    ```
7. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
8. Upgrade Debian Packages.
    ``` bash
    $ sudo apt upgrade
    ```
9. Install ROS Iron Desktop Full.
    ``` bash
    $ sudo apt install ros-iron-desktop
    ```
10. Add aliases to simplify sourcing ROS2 Iron in the future.
    ``` bash
    $ echo "alias source_ros2='source /opt/ros/iron/setup.sh'" >> ~/.bashrc
    $ echo "alias source_ws2='source install/setup.sh'" >> ~/.bashrc
    ```
11. Install rosdep (used for installing dependencies).
    ``` bash
    $ sudo apt install python3-rosdep
    ```
12. Initialize rosdep.
    ``` bash
    $ sudo rosdep init
    ```
13. Update rosdep for ROS Iron.
    ``` bash
    $ rosdep update --rosdistro iron
    ```
14. Update Debian Packages.
    ``` bash
    $ sudo apt update
    ```
15. Upgrade Debian Packages.
    ``` bash
    $ sudo apt upgrade
    ```
16. Remove Non-Required Debian Packages.
    ``` bash
    $ sudo apt autoremove
    ```
17. Reset your VM.
    ``` bash
    $ sudo reboot
    ```

## Connecting to GO1 Robot
### Turning on the Robot
The process to turn on the GO1 Robot can be found in this [video](https://www.youtube.com/watch?v=VbabuAhol0E).

### Ethernet Connection
1. Turn on the robot.
2. Plug the ethernet cable into the port on your device, and on the back of the
    GO1 robot.
3. Open up Terminal on your device.
4. Get the Ethernet Port Name on your device.
    ``` bash
    $ ifconfig
    enp0s25: ... # en* means ethernet - this is the port name we want
    ...
    ```
5. Set a static connection to the robot.
    ``` bash
    $ sudo ifconfig enp0s25 down # replace with your ethernet port name
    $ sudo ifconfig enp0s25 192.168.123.162/24
    $ sudo ifconfig enp0s25 up
    ```
6. Ping to test connection.
    ``` bash
    $ ping -c 3 192.168.123.162
    ```

### WiFi Connection
1. Turn on the robot.
2. The robot's WiFi connection name will start with `"Unitree_Go"`.
3. The robot's WiFi Password is: `00000000` (8 zeros).
4. SSH to test connection.
    ``` bash
    $ ssh pi@192.168.12.1
    123
    ```

## Connecting to GO1 Robot
The following steps will allow you to connect to the Unitree GO1 Robot:
1. Create new Ubuntu 18.04 VM.
    1. See [Ubuntu 18.04 (AMD)](#ubuntu-1804-amd).
    2. See [Connecting to Eduroam](#connecting-to-eduroam).
2. Install ROS Melodic.
    1. See [ROS1 Melodic](#ros1-melodic)
    11. Install ROS Melodic Controllers.
        ``` bash
        $ sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
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
    1. See [Turning on the Robot](#turning-on-the-robot).
    2. Connect via Ethernet or WiFi.
        1. See [Ethernet Connection](#ethernet-connection).
        2. See [WiFi Connection](#wifi-connection).
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
            3. If you want to be able to see the commands and states being
                published in real-time, open a new terminal and input the
                following commands:
                ``` bash
                $ source_ros
                $ rostopic echo /high_cmd # commands from `B`
                $ rostopic echo /high_state # current robot state from `A`
                ```


## Using SNT-ARG Repo
1. Open the [snt-arg unitree_ros repo](https://github.com/snt-arg/unitree_ros).
2. Create an [Ubuntu 22.04 (ARM) VM](#ubuntu-2204-arm).
4. Install ROS2 Iron.
    1. Setup the System + Enable Ubuntu Universe.
        ``` bash
        $ sudo apt update
        $ sudo apt install software-properties-common
        $ sudo add-apt-repository universe
        ```
    2. Add ROS2 GPG Key.
        ``` bash
        $ sudo apt update
        $ sudo apt install curl -y
        $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        ```
    3. Add Repository to Sources List.
        ``` bash
        $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        ```
    4. Install ROS2 Development Tools
        ``` bash
        $ sudo apt update
        $ sudo apt install ros-dev-tools
        ```
    5. Install ROS2 Iron
        ``` bash
        $ sudo apt update
        $ sudo apt upgrade
        $ sudo apt install ros-iron-desktop
        ```
    6. Add aliases to simplify sourcing ROS2 Iron in the future.
        ``` bash
        $ echo "alias source_ros2='source /opt/ros/iron/setup.sh'" >> ~/.bashrc
        $ echo "alias source_ws2='source install/setup.sh'" >> ~/.bashrc
        ```
    7. Restart Ubuntu.
        ``` bash
        $ sudo reboot
        ```
5. Install Unitree ROS.
    ``` bash
    $ sudo apt install ros-iron-unitree-ros
    ```
6. Create a Workspace for this Repo.
    ``` bash
    $ mkdir -p ~/unitree_ws/src
    $ cd ~/unitree_ws/src
    $ git clone --recurse-submodules https://github.com/snt-arg/unitree_ros.git
    ```
7. Build the Workspace for this Repo.
    ``` bash
    $ cd ~/unitree_ws
    $ source_ros2 # source /opt/ros/iron/setup.sh
    $ colcon build --symlink-install
    $ source_ws2 # source install/setup.sh
    ```
8. Connect to the GO1 robot over WiFi + test using the `ssh` command.
9. Launch the driver.
    ``` bash
    ros2 launch unitree_ros unitree_driver_launch.py wifi:=true
    ```
10. View the driver topics (in new terminal, whilst driver is running).
    ``` bash
    $ source_ros2
    $ ros2 topic list
    ```
11. Send a Command Velocity (in new terminal, whilst driver is running).
    ``` bash
    $ ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```
12. Control using Teleop (in new terminal, whilst driver is running).
    ``` bash
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## Controlling LEDs
1. Sign in to Unitree GO1 Main Controller.
    ``` bash
    $ ssh unitree@192.168.123.13
    123
    ```
2. Locate Light SDK.
    ``` bash
    cd Unitree/sdk/faceLightSDK_Nano
    ```
3. Build the SDK.
    ``` bash
    mkdir build
    cd build
    cmake ..
    make
    ```
4. Use (this will rotate through pre-defined colours)
    ``` bash
    ./../bin/faceLightClient
    ```

## Reading Camera Data
1. Install Net Tools.
    ``` bash
    sudo apt install net-tools
    ```
1. Download the [Camera Manual](https://www.docs.quadruped.de/projects/go1/html/operation.html).
2. Get the Head Camera working:
    1. Create an SDK Working Directory and install from GitHub.
        ``` bash
        $ mkdir ~/camera_sdk
        $ cd ~/camera_sdk
        $ git clone https://github.com/unitreerobotics/UnitreecameraSDK.git
        ```
    2. Use SCP (Secure Copy Protocol) to transfer the camera sdk to the robot.
        ``` bash
        $ scp -r UnitreecameraSDK unitree@192.168.123.13:/home/unitree
        123
        ```
    3. SSH into the Robot.
        ``` bash
        $ ssh unitree@192.168.123.13
        123
        ```
    4. Stop the Head Camera Processes:
        ``` bash
        ps -aux | grep point_cloud_node | awk '{print $2}' | xargs kill -9
        ps -aux | grep mqttControlNode | awk '{print $2}' | xargs kill -9
        ```
    5. Modify `trans_rect_config.yaml`:
        ``` bash
        $ cd ~/UnitreecameraSDK
        $ vim trans_rect_config.yaml
        ```
        1. Update the `IpLastSegment.data` Value:
            - Original: `[ 15. ]`
            - New: `[ 126. ]` # 13, 100, 255, 252
        2. Update the `DeviceNode.data` Value:
            - Original: `[ 0. ]`
            - New: `[ 1. ]`
    6. Build the Camera SDK.
        ``` bash
        $ mkdir build
        $ cd build
        $ cmake ..
        $ make
        ```
    7. Run the Image Transmitter.
        ``` bash
        $ cd ..
        $ ./bins/example_putImagetrans
        ```
    8. Open a new Terminal, and go to the original camera SDK.
        ``` bash
        $ cd ~/camera_sdk/UnitreecameraSDK # on your machine
        ```
    9. Edit the `example_getimagetrans.cc` File.
        ``` bash
        $ nano examples/example_getimagetrans.cc
        ```
        Update the start of the `main` function so it looks like this:
        ``` cpp
        int main(int argc, char** argv)
        {
            std::string IpLastSegment = "126";
            int cam = 1;
        ```
    10. Build the SDK.
        ``` bash
        $ mkdir build
        $ cd build
        $ cmake ..
        $ make
        ```
    11. Run the Image Receiver.
        ``` bash
        $ cd ..
        $ ./bins/example_getimagetrans
        ```
1. Connect to a single camera.
    1. Install GLUT.
        ``` bash
        sudo apt install freeglut3-dev
        ```
    2. Create a new workspace, clone the Camera SDK, and build.
        ``` bash
        $ mkdir ~/camera_sdk
        $ cd ~/camera_sdk
        $ git clone https://github.com/unitreerobotics/UnitreecameraSDK
        $ cd UnitreecameraSDK
        $ mkdir build
        $ cd build
        $ cmake ..
        $ make
        ```
    4. Login to Main GO1 Controller.
        ``` bash
        $ ssh unitree@192.168.123.13
        >>> 123

        $ cd Unitree/sdk/
    3. Get Camera Raw Frame.
        ``` bash
        $ cd ~/camera_sdk/UnitreecameraSDK
        ./bins/example_getRawFrame
        ```