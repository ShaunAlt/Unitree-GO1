# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Unitree GO1 Robot Interface Model - High Level
-
This module contains object definitions that simplify the control and
interaction with the Unitree GO1 Robot when running the
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel` command in
another terminal whilst connected to the Unitree GO1 Robot.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for ros object definitions
from .model_ros import (
    ROS_Node,
    ROS_Publisher,
    ROS_Subscriber,
)

# used for unitree go1 message definitions
import unitree_legged_msgs # type: ignore

# used for type hinting
from typing import (
    Any,
    TYPE_CHECKING,
)


# =============================================================================
# High Level GO1 Robot State
# =============================================================================
class State_HighLevel(object):
    '''
    High Level GO1 Robot State
    -
    Contains the data for the current high-level state of the robot.

    Attributes
    -
    - _head : `tuple[int, int]`
        - TODO.
    - _levelFlag : `int`
        - TODO.
    - 
    '''


# =============================================================================
# High Level GO1 Controller
# =============================================================================
class Controller_HighLevel(ROS_Node):
    '''
    High Level GO1 Controller
    -
    Contains the attributes and methods used to simplify the high-level control
    of the GO1 robot.

    Attributes
    -
    None

    Constants
    -
    None

    Methods
    -
    - __init__(name, debug=False) : `None`
        - Constructor Method.
        - Used to create a new high level GO1 controller.

    Properties
    -
    None
    '''

    # ===========
    # Constructor
    def __init__(self, name: str, debug: bool = False) -> None:
        '''
        High Level GO1 Controller Constructor
        -
        Creates a new high level GO1 controller.

        Parameters
        -
        - name : `str`
            - Unique identifying name of the ROS node.
        - debug : `bool`
            - Flag for whether or not debug logging is enabled.

        Returns
        -
        None
        '''

        # run ros node initialization
        super().__init__(name, debug)

        # create controller publisher - used to send commands to the robot
        self._pub = ROS_Publisher(
            name = '/highcmd',
            data_class = unitree_legged_msgs.msg.HighCmd
        )

        # create controller subscriber - used to read robot data
        self._sub = ROS_Subscriber(
            name = '/highstate',
            data_class = unitree_legged_msgs.msg.HighState
        )

        # initialize robot state
        self.state = None


# =============================================================================
# End of File
# =============================================================================
