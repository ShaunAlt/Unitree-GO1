# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Unitree GO1 Robot Interface Model - Messages
-
This module contains object definitions that can be used to store and
manipulate the data received from the Unitree GO1 Robot.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for creating enumerable objects
from enum import Enum

# used for unitree go1 message definitions
import unitree_legged_msgs # type: ignore

# used for type hinting
from typing import (
    Any,
    TYPE_CHECKING,
)


# =============================================================================
# Parent Message Definition
# =============================================================================
class MSG(object):
    '''
    Parent Message
    -
    Contains the common attributes and methods for all custom message objects.

    Attributes
    -
    None

    Constants
    -
    None

    Methods
    -
    - from_msg(msg) : `MSG`
        - Class Method.
        - Creates a new instance of the message from a ROS message object.
    - to_msg() : `Any`
        - Instance Method.
        - Converts the current message object to a ROS message object.

    Properties
    -
    None
    '''

    # =======================
    # Create From ROS Message
    @classmethod
    def from_msg(cls, msg: Any) -> 'MSG':
        '''
        Create From ROS Message
        -
        Creates a new instance of the message from a ROS message object.

        Parameters
        -
        - msg : `Any`
            - ROS message object.

        Returns
        -
        - `MSG`
            - New message object.
        '''

        raise NotImplementedError(
            f'MSG.from_msg() is not implemented in {cls}'
        )
    
    # ======================
    # Convert to ROS Message
    def to_msg(self) -> Any:
        '''
        Convert to ROS Message
        -
        Converts the current message object to a ROS message object.

        Parameters
        -
        None

        Returns
        -
        - `Any`
            - ROS message object.
        '''

        raise NotImplementedError(
            f'MSG().to_msg() is not implemented in {self.__class__}'
        )


# =============================================================================
# Robot Messages
# =============================================================================

# ===================
# BMS Command Message
class MSG_BmsCmd(MSG):
    '''
    Battery-Management-System Command Message
    -
    Contains the data for giving commands to the BMS in the robot.

    Maps to `unitree_legged_msgs.msg.BmsCmd`.

    Attributes
    -
    - _off : `bool` (`uint8`)
        - Flag for if to signal that the battery should shut down.
    - _reserve : `tuple[int, int, int]` (`uint8[3]`)
        - Not currently implemented, spare bits saved for future use.

    Constants
    -
    None
    
    Methods
    -
    - __init__(off=False) : `None`
        - Constructor Method.
        - Creates a new BMS Command Message.
    - to_msg() : `unitree_legged_msgs.msg.BmsCmd`
        - `MSG` Instance Method.
        - Converts the current message object to a ROS message object.

    Properties
    -
    - off : `bool`
        - Readonly.
        - Flag for if to signal that the battery should shut down.
    '''

    # ===========
    # Constructor
    def __init__(self, off: bool = False) -> None:
        '''
        Battery-Management-System Command Message Constructor
        -
        Creates a new BMS Command Message.

        Parameters
        -
        - off : `bool`
            - Flag for if to signal that the battery should shut down.

        Returns
        -
        None
        '''

        # set `off` value
        self._off: bool = off
        ''' Flag for if to signal that the battery should shut down. '''

    # ===================
    # Property - Off Flag
    @property
    def off(self) -> bool:
        ''' Flag for if to signal that the battery should shut down. '''
        return self._off
    
    # ======================
    # Convert to ROS Message
    def to_msg(self) -> unitree_legged_msgs.msg.BmsCmd:
        return unitree_legged_msgs.msg.BmdCmd(
            off = 0xA5 if self.off else 0,
            reserve = (0, 0, 0)
        )

# =================
# BMS State Message
class MSG_BmsState(MSG):
    pass

# =================
# Cartesian Message
class MSG_Cartesian(MSG):
    '''
    Cartesian Message
    -
    Contains the data for a single 3-DOF point.

    Maps to `unitree_legged_msgs.msg.Cartesian`.

    Attributes
    -
    - _x : `float` (`float32`)
        - X-Coordinate as a floating point.
    - _y : `float` (`float32`)
        - Y-Coordinate as a floating point.
    - _z : `float` (`float32`)
        - Z-Coordinate as a floating point.

    Constants
    -
    None

    Methods
    -
    - __init__(x, y, z) : `None`
        - Constructor Method.
        - Creates a new Cartesian message object.
    - from_msg(msg) : `MSG`
        - Class Method.
        - Creates a new instance of the message from a ROS message object.

    Properties
    -
    None
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            x: float,
            y: float,
            z: float
    ) -> None:
        '''
        Cartesian Message Constructor
        -
        Creates a new Cartesian message object.

        Parameters
        -
        - x : `float`
            - X-coordinate as a floating point.
        - y : `float`
            - Y-Coordinate as a floating point.
        - z : `float`
            - Z-Coordinate as a floating point.

        Returns
        -
        None
        '''

        self._x: float = x
        ''' X-Coordinate as a floating point. '''
        self._y: float = y
        ''' Y-Coordinate as a floating point. '''
        self._z: float = z
        ''' Z-Coordinate as a floating point. '''

    # ===============================
    # `MSG` - Create From ROS Message
    @classmethod
    def from_msg(
            cls,
            msg: unitree_legged_msgs.msg.Cartesian
    ) -> 'MSG_Cartesian':
        return cls(
            x = msg.x,
            y = msg.y,
            z = msg.z
        )

# ====================
# High Command Message
class MSG_HighCmd(MSG):
    '''
    High Level Command Message
    -
    Contains the data for sending a high-level command to the robot.

    Maps to `unitree_legged_msgs.msg.HighCmd`.

    Attributes
    -
    - _mode : `MSG_HighCmd.Mode` (`uint8`)
        - Run mode of the robot.
    - _gaitType : `MSG_HighCmd.GaitType` (`uint8`)
        - Gait type of the robot.
    - _speedLevel : `MSG_HighCmd.SpeedLevel` (`uint8`)
        - Speed level of the robot. Only used if `_mode` is
            `MSG_HighCmd.WALK_POS`.
    - _footRaiseHeight : `float` (`float32`)
        - Measured in meters. Swing foot height adjustment from default swing
            height delta. Default value is `0.08`.
    - _bodyHeight : `float` (`float32`)
        - Measured in meters. Body height adjustment from default body height
            delta. Default value is `0.28`.
    - _position : `tuple[float, float]` (`float32[2]`)
        - Measured in meters. Desired x and y position in the inertial frame,
            which is established at the beginning of the sport mode. Only used
            if `_mode` is `MSG_HighCmd.WALK_POS`.
    - _euler : `tuple[float, float, float]` (`float32[3]`)
        - Measured in radians. Desired roll-pitch-yaw Euler angle, with
            euler[0] = Roll, euler[1] = Pitch, euler[2] = Yaw. RPY is used if
            `_mode` is `MSG_HighCmd.STAND` to set target orientation. The yaw
            value is used if `_mode` is `MSG_HighCmd.WALK_POS` as target yaw
            angle.
    - _velocity : `tuple[float, float]` (`float32[2]`)
        - Measured in meters per second. Desired robot forward speed and side
            speed in the body frame. Velocity setting is used when `_mode` is
            `MSG_HighCmd.WALK_VEL` as target linear velocity.
    - _yawSpeed : `float` (`float32`)
        - Desired rotational yaw speed. Yaw speed setting is used when `_mode`
            is `MSG_HighCmd.WALK_VEL` as target rotational speed.
    - _bms : `MSG_BmsCmd` (`unitree_legged_msgs.msg.BmsCmd`)
    - _led : `tuple[MSG_Led, MSG_Led, MSG_Led, MSG_Led]` (`unitree_legged_msgs.msg.LED[4]`)

    Constants
    -
    - GaitType : `Enum`
        - Collection of valid gait types.
    - Modes : `Enum`
        - Collection of valid modes.
    - SpeedLevel : `Enum`
        - Collection of speed levels.

    Methods
    -
    - __init__(self, ) : `None`
        - Constructor Method.
        - Creates a new high level command message object.
    - to_msg() : `unitree_legged_msgs.msg.HighCmd`
        - `MSG` Instance Method.
        - Converts the current message object to a ROS message object.

    Properties
    -
    None
    '''

    # =========
    # Constants
    class GaitType(Enum):
        ''' Collection of valid gait types. '''

        IDLE = 0
        ''' Idle. '''
        WALK = 1
        ''' Trot walking. '''
        RUN = 2
        ''' Trot running. '''
        STAIRS = 3
        ''' Stairs climbing. '''
        OBSTACLE = 4
        ''' Trot obstacle. '''

    class Mode(Enum):
        ''' Collection of valid modes. '''

        IDLE = 0
        ''' Idle. Will disregard other control commands. '''
        STAND = 1
        ''' Standing, in force control. Controlled by `euler` and `bodyHeight`
            commands. '''
        WALK_VEL = 2
        ''' Walking, following target velocity. Controlled by `velocity`,
            `yawSpeed`, `bodyHeight`, and `footRaiseHeight` commands. '''
        WALK_POS = 3
        ''' Walking, following target position, reserve for future release. '''
        WALK_PATH = 4
        ''' Walking, following a given path, reserve for future release. '''
        STAND_DOWN = 5
        ''' Stand down, in position control. This should only ever be used as a
            transitional state. Do not keep this state for a long time. '''
        STAND_UP = 6
        ''' Stand up, in position control. '''
        DAMP = 7
        ''' Damping mode, all motors. '''
        RECOVERY = 8
        ''' Recovery mode. '''
        BACKFLIP = 9
        ''' backflip. '''
        JUMPYAW = 10
        ''' jumpYaw. '''
        STRAIGHTHAND = 11
        ''' straightHand. '''
        DANCE1 = 12
        ''' dance1. '''
        DANCE2 = 13
        ''' dance2. '''

    class SpeedLevel(Enum):
        ''' Collection of speed levels. '''

        LOW = 0
        ''' Default low speed. '''
        MEDIUM = 1
        ''' Default medium speed. '''
        HIGH = 2
        ''' Default high speed. '''

    # ===========
    # Constructor
    def __init__(
            self,
            mode: int,
            gait_type: int,

            ''' TODO '''
    ) -> None:
        '''
        
        '''

    # Convert to 
    def to_msg(self) -> unitree_legged_msgs.msg.HighCmd:
        return unitree_legged_msgs.msg.HighCmd(
            head = (0xfe, 0xef), # uint8[2]
            levelFlag = 0xee, # uint8
            frameReserve = 0, # uint8
            SN = (0, 0), # uint32[2]
            version = (0, 0), # uint32[2]
            bandWidth = 0, # uint16
            mode = self.mode,
            gaitType = self.gaitType,
            speedLevel = self.speedLevel,
            footRaiseHeight = self.footRaiseHeight,
            bodyHeight = self.bodyHeight,
            position = self.position,
            euler = self.euler,
            velocity = self.velocity,
            yawSpeed = self.yawSpeed,
            bms = self.bms.to_msg(),
            led = (
                l.to_msg()
                for l in self.led
            ),
            wirelessRemote = (0 for _ in range(40)),
            reserve = 0,
            crc = 0
        )

# ==================
# High State Message
class MSG_HighState(MSG):
    '''
    High State Message
    -
    Contains the data for the current high-level state of the robot.
    
    Maps to `unitree_legged_msgs.msg.HighState`.

    Attributes
    -
    - _head : `tuple[int, int]`
        - TODO.
    - _levelFlag : `int`
        - TODO.
    - 
    '''

# ===========
# IMU Message
class MSG_Imu(MSG):
    pass

# ===========
# LED Message
class MSG_Led(MSG):
    '''
    LED Message
    -
    Contains the data for setting the RGB values of a particular LED on the
    robot.

    Maps to `unitree_legged_msgs.msg.LED`.

    Attributes
    -
    - _rgb : `tuple[int, int, int]` (`uint8, uint8, uint8`)
        - Red, Green, Blue `uint8` codes in sequence.

    Constants
    -
    - AQUA : `tuple[int, int, int]`
        - RGB code for an Aqua LED.
    - BLUE : `tuple[int, int, int]`
        - RGB code for a Blue LED.
    - GREEN : `tuple[int, int, int]`
        - RGB code for a Green LED.
    - PURPLE : `tuple[int, int, int]`
        - RGB code for a Purple LED.
    - RED : `tuple[int, int, int]`
        - RGB code for a Red LED.
    - YELLOW : `tuple[int, int, int]`
        - RGB code for a Yellow LED.

    Methods
    -
    - __init__(self, r, g, b) : `None`
        - Constructor Method.
        - Creates a new LED message object.
    - to_msg() : `unitree_legged_msgs.msg.LED`
        - `MSG` Instance Method.
        - Converts the current message object to a ROS message object.

    Properties
    -
    - b : `int`
        - Readonly.
        - Blue value in the RGB code.
    - g : `int`
        - Readonly.
        - Green value in the RGB code.
    - r : `int`
        - Readonly.
        - Red value in the RGB code.
    '''

    # =========
    # Constants
    AQUA = (0, 255, 255)
    ''' RGB code for an Aqua LED. '''
    BLUE = (0, 0, 255)
    ''' RGB code for a Blue LED. '''
    GREEN = (0, 255, 0)
    ''' RGB code for a Green LED. '''
    PURPLE = (255, 0, 255)
    ''' RGB code for a Purple LED. '''
    RED = (255, 0, 0)
    ''' RGB code for a Red LED. '''
    YELLOW = (255, 255, 0)
    ''' RGB code for a Yellow LED. '''

    # ===========
    # Constructor
    def __init__(self, r: int, g: int, b: int) -> None:
        '''
        LED Message Constructor
        -
        Creates a new LED message object.

        Parameters
        -
        - r : `int`
            - Red value in the RGB code.
        - g : `int`
            - Green value in the RGB code.
        - b : `int`
            - Blue value in the RGB code.

        Returns
        -
        None
        '''

        # set attributes
        self._rgb: tuple[int, int, int] = (r, g, b)
        ''' Red, Green, Blue `uint8` codes in sequence. '''

    # ======================
    # Parameter - Blue Value
    @property
    def b(self) -> int:
        ''' Blue value in the RGB code. '''
        return self._rgb[2]
    
    # =======================
    # Parameter - Green Value
    @property
    def g(self) -> int:
        ''' Green value in the RGB code. '''
        return self._rgb[1]
    
    # =====================
    # Parameter - Red Value
    @property
    def r(self) -> int:
        ''' Red value in the RGB code. '''
        return self._rgb[0]

    # ======================
    # Convert to ROS Message
    def to_msg(self) -> unitree_legged_msgs.msg.LED:
        return unitree_legged_msgs.msg.LED(
            r = self.r,
            g = self.g,
            b = self.b
        )

# ===================
# Low Command Message
class MSG_LowCmd(MSG):
    pass

# =================
# Low State Message
class MSG_LowState(MSG):
    pass

# =====================
# Motor Command Message
class MSG_MotorCmd(MSG):
    pass

# ===================
# Motor State Message
class MSG_MotorState(MSG):
    pass


# =============================================================================
# End of File
# =============================================================================
