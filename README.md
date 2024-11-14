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
- [Controlling the GO1 Robot](#controlling-the-go1-robot)
    - [Controlling Movement in ROS1](#controlling-movement-in-ros1)
    - [Controlling Movement in ROS2](#controlling-movement-in-ros2)
    - [Controlling LEDs](#controlling-leds)
    - [Controlling Cameras](#controlling-cameras)

## Contributors
This project was created by Shaun Altmann (shaun.altmann@deakin.edu.au).

## Dependencies
- Git Submodules:
    - [snt-arg/unitree_ros](https://github.com/snt-arg/unitree_ros)
    - [unitreerobotics/unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)
    - [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)
    - [unitreerobotics/unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)
    - [unitreerobotics/UnitreecameraSDK](https://github.com/unitreerobotics/UnitreecameraSDK)
- Previous GO1 Robot Projects:
    - [alkatra/PLT_Fusion](https://github.com/alkatra/PTL_Fusion)
    - [thieulong/Unitree-Go1-EDU](https://github.com/thieulong/Unitree-Go1-EDU)
- [Go1 Manual](https://www.docs.quadruped.de/projects/go1/html/operation.html)
- [Go1 Slack GitHub](https://github.com/MAVProxyUser/YushuTechUnitreeGo1)
- [Go1 Description GitHub](https://github.com/navneethooda/go1_description)
- [LiDAR Unitree Support](https://support.unitree.com/home/en/developer/LiDAR_service)
- [Camera Tutorial](https://www.youtube.com/watch?v=nafv21HeeEM)

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

## Controlling the GO1 Robot
### Controlling Movement in ROS1
Dependencies:
- [unitreerobotics/unitree_ros](submodules/unitreerobotics/unitree_ros/)
- [unitreerobotics/unitree_ros_to_real](submodules/unitreerobotics/unitree_ros_to_real/)
- [unitreerobotics/unitree_legged_sdk](submodules/unitreerobotics/unitree_legged_sdk/)

Steps:
1. Create an [Ubuntu 18.04 (AMD)](#ubuntu-1804-amd) VM.
2. Install [ROS1 Melodic](#ros1-melodic).
3. Install ROS Melodic Controllers.
    ``` bash
    $ sudo apt install ros-melodic-controller-interface
    $ sudo apt install ros-melodic-gazebo-ros-control
    $ sudo apt install ros-melodic-joint-state-controller
    $ sudo apt install ros-melodic-effort-controllers
    $ sudo apt install ros-melodic-joint-tragectory-controller
    ```
4. Create a ROS Workspace.
    ``` bash
    $ source_ros
    $ mkdir -p ~/ros_ws/src
    $ cd ~/ros_ws/src
    ```
5. Install the Unitree GO1 Repositories.
    ``` bash
    $ git clone https://github.com/unitreerobotics/unitree_ros -b master
    $ git clone https://github.com/unitreerobotics/unitree_ros_to_real -b master
    $ git clone https://github.com/unitreerobotics/unitree_legged_sdk -b go1
    ```
6. Install Project Dependencies.
    ``` bash
    $ cd ~/ros_ws
    $ rosdep install --from-paths src -y --ignore-src --rosdistro melodic
    ```
7. Build the Workspace.
    ``` bash
    $ catkin_make
    ```
8. Connect to the GO1 Robot over [Ethernet](#ethernet-connection).
9. Run the example walk.
    1. In one terminal, run the following commands:
        ``` bash
        $ cd ~/ros_ws
        $ source_ws
        $ roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
        ```
    2. Open a new terminal and run the following commands:
        ``` bash
        $ cd ~/ros_ws
        $ source_ws
        $ rosrun unitree_legged_real ros_example_walk
        ```
    3. (Optional) Open a new terminal and run any one of the following commands
        to view the data being sent (after first using `source_ros`).
        - View a list of ROS topics.
            ``` bash
            $ rostopic list
            ```
        - See the high-level commands being sent from the device to the robot.
            ``` bash
            $ rostopic echo /high_cmd
            ```
        - See the high-level state of the robot being sent to the device.
            ``` bash
            $ rostopic echo /high_state
            ```

### Controlling Movement in ROS2
Dependencies:
- [snt-arg/unitree_ros](submodules/snt-arg/unitree_ros/)

Steps:
1. Create an [Ubuntu 22.04 (ARM)](#ubuntu-2204-arm) VM.
2. Install [ROS2 Iron](#ros2-iron).
3. Install Unitree ROS.
    ``` bash
    $ sudo apt install ros-iron-unitree-ros
    ```
4. Create a ROS2 Workspace.
    ``` bash
    $ source_ros2
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    ```
5. Install the Unitree ROS Repository.
    ``` bash
    $ git clone --recurse-submodules https://github.com/snt-arg/unitree_ros.git
    ```
6. Install Project Dependencies.
    ``` bash
    $ cd ~/ros2_ws
    $ rosdep install --from-paths src -y --ignore-src --rosdistro iron
7. Build the Workspace.
    ``` bash
    $ colcon build --symlink-install
    ```
8. Connect to the GO1 Robot over [WiFi](#wifi-connection).
9. Run the Driver with Teleop Keyboard.
    1. In one terminal, run the following commands:
        ``` bash
        $ cd ~/ros2_ws
        $ source_ws2
        $ ros2 launch unitree_ros unitree_driver_launch.py wifi:=true
        ```
    2. Open a new terminal and run the following commands:
        ``` bash
        $ source_ros2
        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
        ```
    3. (Optional) Open a new terminal and run any one of the following commands
        to view the data being sent (after first using `source_ros2`).
        - View a list of ROS2 topics.
            ``` bash
            $ ros2 topic list
            ```
        - See the command velocities being sent to the robot.
            ``` bash
            $ ros2 topic echo /cmd_vel
            ```

### Controlling LEDs
> [!NOTE]
> This section currently contains a lot of guess-work. I have not yet managed
> to get the LED control working, and as such this section contains a list of
> different things I have tried.

#### Multi-Colour Blinking Face-Lights
The following steps will make the face lights on the robot flash different
colours:
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

### Controlling Cameras
> [!NOTE]
> This section currently contains a lot of guess-work. I have not yet managed
> to get the camera control working, and as such this section contains a list
> of different things I have tried.
#### Getting Raw Camera Data
This is not working.
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

#### Get Camera Data from Robot to VM - 1
This is not working.

Download the [Camera Manual](https://www.docs.quadruped.de/projects/go1/html/operation.html).

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

#### Camera Data Transmission - YouTube
Following the tutorial in this [Camera Tutorial](https://www.youtube.com/watch?v=nafv21HeeEM).

1. SSH into the Robot.
    ``` bash
    $ ssh unitree@192.168.123.13 # ip of the head camera
    123 # password
    ```
2. Go to the `UnitreecameraSDK` and remove the original version.
    ``` bash
    $ cd Unitree/sdk
    $ rm -rf UnitreecameraSDK
    $ exit
    ```
3. Use SCP to copy the `UnitreecameraSDK` git repository into where the
    original version was stored.
    ``` bash
    $ mkdir ~/camera_sdk
    $ cd ~/camera_sdk
    $ git clone https://github.com/unitreerobotics/UnitreecameraSDK.git
    $ scp -r UnitreecameraSDK unitree@192.168.123.13:/home/unitree/Unitree/sdk
    123
    ```
4. Get the IP address of the machine receiving the camera data. (This machine
    should be connected to the WiFi of the robot, meaning if you are using a
    VM you will need to use a WiFi bridged adapter instead of the normal shared
    network).
    ``` bash
    $ ifconfig
    ...
        inet 192.168.123.xxx # you need to find the `xxx`
    ```
5. SSH into the Robot.
    ``` bash
    $ ssh unitree@192.168.123.13
    123
    ```
6. Edit the `trans_rect_config.yaml` in the UnitreecameraSDK Directory.
    ``` bash
    $ cd ~/Unitree/sdk/UnitreecameraSDK
    $ vim trans_rect_config.yaml
    ```
7. Change the `IpLastSegment.data` value to whatever `xxx` was from the
    `ifconfig` command.
8. Re-Create and Redirect into the `build` directory.
    ``` bash
    $ rm -rf build
    $ mkdir build
    $ cd build
    ```
9. Make the Repository.
    ``` bash
    $ cmake ..
    $ make
    ```
10. Run the example image transmitter.
    ``` bash
    $ cd ..
    $ ./bins/example_putImagetrans
    ```
    - If you run into any issues, try using the following commands:
        ``` bash
        $ ps -aux | grep point_cloud_node | awk '{print $2}' | xargs kill -9
        $ ps -aux | grep mqttControlNode | awk '{print $2}' | xargs kill -9
        ```
    - If that doesn't work, restart the head camera controller. This will force
        you to quit the SSH session, so you will then need to log back in.
        ``` bash
        $ sudo reboot
        ```
11. Open a new terminal, and go to the original camera sdk.
    ``` bash
    $ cd ~/camera_sdk/UnitreecameraSDK/
    ```
12. Edit the `example_getimagetrans.cc` File.
    ``` bash
    $ nano examples/example_getimagetrans.cc
    ```
    Update the start of the `main` function so it looks like this:
    ``` cpp
    int main(int argc, char** argv)
    {
        std::string IpLastSegment = "xxx"; // equal to `xxx` from `ifconfig`
        int cam = 1;
    ```
12. Re-Create and Redirect into the `build` directory.
    ``` bash
    $ rm -rf build
    $ mkdir build
    $ cd build
    ```
13. Make the Repository.
    ``` bash
    $ cmake ..
    $ make
    ```
14. Run the example image receiver.
    ``` bash
    $ ./bins/example_getimagetrans
    ```

#### Camera Data Transmission - From Main Controller
1. Delete `UnitreecameraSDK` from main controller if there.
    ``` bash
    $ ssh pi@192.168.12.1 # wifi connection
    123
    $ cd Unitree/sdk
    $ ls -a # check if UnitreecameraSDK is there
    $ rm -rf UnitreecameraSDK # if required
    $ exit
    ```
2. Create new `camera_sdk` workspace and upload new `UnitreecameraSDK`.
    ``` bash
    $ rm -rf ~/camera_sdk
    $ mkdir ~/camera_sdk
    $ cd ~/camera_sdk
    $ git clone https://github.com/unitreerobotics/UnitreecameraSDK.git
    $ scp -r UnitreecameraSDK pi@192.168.12.1:/home/pi/Unitree/sdk
    123
3. Open new terminal and get IP of current machine.
    ``` bash
    $ ifconfig
    ...
        192.168.12.XXX # want to know this
    ```
4. Edit `trans_rect_config.yaml` in Robot's Camera SDK.
    ``` bash
    $ ssh pi@192.168.12.1
    123
    $ cd Unitree/sdk/UnitreecameraSDK
    $ nano trans_rect_config.yaml
    >>> IpLastSegment.data = XXX # from `ifconfig`
    >>> DeviceNode.data = 1
    ```
5. Build Package.
    ``` bash
    $ rm -rf build
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ cd ..
    ```
6. Run Image Transmitter.
    ``` bash
    $ ./bins/example_putImagetrans
    ```
