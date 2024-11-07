cmake_minimum_required(VERSION 3.0.2)
project(go1_ros_model)

# package dependencies
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    unitree_legged_msgs
)

# use `setup.py` to configure the package
catkin_python_setup()

# register the package with catkin
catkin_package()

# header file locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)